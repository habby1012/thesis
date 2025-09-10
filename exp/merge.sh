#!/bin/bash
set -e

if [ -z "$1" ]; then
  echo "Usage: $0 <realpath to stat.csv or its directory>"
  exit 1
fi

INPUT=$1

if [ -d "$INPUT" ]; then
  REALPATH="$INPUT/stat.csv"
else
  REALPATH="$INPUT"
fi

OUT_DIR=$(dirname "$REALPATH")
BASENAME=$(basename "$REALPATH")

REMOTES=("100.90.230.109" "100.72.182.68")
TMP_DIR=$(mktemp -d)

cp "$REALPATH" "$TMP_DIR/local_${BASENAME}"

for ip in "${REMOTES[@]}"; do
    scp "${ip}:${REALPATH}" "$TMP_DIR/${ip}_${BASENAME}" || true
done

TMP_DIR="$TMP_DIR" OUT_DIR="$OUT_DIR" BASENAME="$BASENAME" python3 - <<'EOF'
import pandas as pd
import glob, os

tmp_dir = os.environ["TMP_DIR"]
out_dir = os.environ["OUT_DIR"]
basename = os.environ["BASENAME"]

files = glob.glob(os.path.join(tmp_dir, f"*_{basename}"))
dfs = []
for f in files:
    try:
        df = pd.read_csv(f)
        dfs.append(df)
    except Exception as e:
        print("Skip", f, "->", e)

if not dfs:
    raise RuntimeError("No stat.csv files collected")

merged = pd.concat(dfs, ignore_index=True)

priority_order = {
    'Critical': 0,
    'Soft Real-Time': 1,
    'Perception': 2,
    'Best-Effort': 3,
    'Unknown': 4,
}
merged["class_order"] = merged["class"].map(priority_order).fillna(99).astype(int)
merged = merged.sort_values(["class_order", "topic"], kind="mergesort").drop(columns=["class_order"])

out_path = os.path.join(out_dir, "merged_stat.csv")
merged.to_csv(out_path, index=False)
print("Merged into", out_path)
EOF

rm -rf "$TMP_DIR"

