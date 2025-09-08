import argparse, pandas as pd, numpy as np

def _has_header(path):
    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
        return 'topic' in f.readline().lower()

def _norm_topic(s: str) -> str:
    s = ('' if s is None else str(s)).strip()
    while s.startswith('/'): s = s[1:]
    while '//' in s: s = s.replace('//','/')
    return s

def load_latency(p):
    hdr = 0 if _has_header(p) else None
    names = None if hdr == 0 else ['topic','seq','latency_ns']
    df = pd.read_csv(p, header=hdr, names=names, usecols=[0,1,2],
                     dtype={0:'string',1:'Int64',2:'Int64'}, engine='c')
    df.columns = ['topic','seq','latency_ns']
    df = df.dropna(subset=['topic','latency_ns']).copy()
    df['topic'] = df['topic'].astype('string')
    df['latency_ns'] = df['latency_ns'].astype('int64')
    df['topic_norm'] = df['topic'].map(_norm_topic)
    df['_row'] = np.arange(len(df), dtype=np.int64)
    return df

def load_policy(p):
    pol = pd.read_csv(p, header=None,
                      names=['topic','module','class','frequency','size','bandwidth'],
                      dtype={'topic':'string','class':'string'}, engine='c')
    pol['topic_norm'] = pol['topic'].map(_norm_topic)
    pol = pol.drop_duplicates(subset=['topic_norm'])
    return pol[['topic_norm','class']]

def loss_rate_from_seq(g: pd.DataFrame) -> float:
    s = g['seq'].dropna().astype('int64').to_numpy()
    if s.size <= 1: return np.nan
    u = pd.unique(s)
    expected = int(u[-1] - u[0] + 1)
    if expected <= 0: return np.nan
    received = int(len(u))
    miss = max(expected - received, 0)
    return float(miss) / float(expected)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--in', dest='in_path', required=True)
    ap.add_argument('--policy', dest='policy_path', required=True)
    ap.add_argument('--out', dest='out_path', required=True)
    ap.add_argument('--deadline-ns', dest='deadline_ns', type=int, default=1_000_000)
    ap.add_argument('--warmup-n', dest='warmup_n', type=int, default=0)
    a = ap.parse_args()

    lat = load_latency(a.in_path)
    pol = load_policy(a.policy_path)
    m = lat.merge(pol, on='topic_norm', how='left')
    m['class'] = m['class'].fillna('Unknown')

    rows = []
    for topic, g in m.groupby('topic', sort=False):
        g = g.sort_values('_row', kind='mergesort')
        if a.warmup_n > 0:
            if len(g) > a.warmup_n:
                g = g.iloc[a.warmup_n:].copy()
            else:
                continue
        if g.empty: continue

        cls = str(g['class'].iloc[0]) if 'class' in g.columns else 'Unknown'
        L_ns = g['latency_ns'].to_numpy(np.int64)
        L_ms = L_ns.astype(np.float64) / 1e6

        loss_rate = loss_rate_from_seq(g)

        avg_ms = float(np.mean(L_ms))
        p95_ms = float(np.percentile(L_ms, 95))
        p99_ms = float(np.percentile(L_ms, 99))

        jitter_std_ms = float(np.std(L_ms, ddof=0))
        ipdv_std_ms = float(np.std(np.diff(L_ms), ddof=0)) if L_ms.size > 1 else np.nan

        miss = float(np.mean(L_ns > a.deadline_ns)) if cls.lower()=='critical' else np.nan

        rows.append({
            'topic': topic,
            'class': cls,
            'count': int(L_ms.size),
            'avg_latency_ms': round(avg_ms, 3),
            'p95_latency_ms': round(p95_ms, 3),
            'p99_latency_ms': round(p99_ms, 3),
            'jitter_std_ms': round(jitter_std_ms, 3),
            'ipdv_std_ms': round(ipdv_std_ms, 3) if not np.isnan(ipdv_std_ms) else np.nan,
            'deadline_miss_rate_1ms': round(miss, 6) if not np.isnan(miss) else np.nan,
            'loss_rate': round(loss_rate, 6) if not np.isnan(loss_rate) else np.nan,
        })

    out = pd.DataFrame(rows).sort_values('topic', kind='mergesort')
    out.to_csv(a.out_path, index=False)

if __name__ == '__main__':
    main()

