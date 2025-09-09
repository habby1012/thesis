import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
import csv

class CsvTopicSubscriber(Node):
    def __init__(self):
        super().__init__('csv_topic_subscriber')
        self.subs = []
        csv_path = '/home/newslab/repos/thesis/src/subscriber/subscriber/config/metadata_table.csv'
        self.target_topics = self.read_target_topics(csv_path)
        # self.get_logger().info(f'Target topics: {self.target_topics}')
        self.timer = self.create_timer(1.0, self.scan_and_subscribe)

    def read_target_topics(self, csv_path):
        topics = set()
        with open(csv_path, newline='') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                if len(row) < 3:
                    continue
                if row[2].strip() == 'Vehicle/System':
                    t = row[0].strip()
                    if not t.startswith('/'):
                        t = '/' + t
                    topics.add(t)
        return topics

    def scan_and_subscribe(self):
        topics_and_types = self.get_topic_names_and_types()
        for topic_name, types in topics_and_types:
            if topic_name not in self.target_topics:
                continue
            if topic_name in [s.topic_name for s in self.subs]:
                continue
            try:
                msg_type = get_message(types[0])
                sub = self.create_subscription(
                    msg_type, topic_name, self.generic_callback, 10)
                self.subs.append(sub)
                # self.get_logger().info(f'Subscribed: {topic_name} ({types[0]})')
            except Exception as e:
                self.get_logger().warn(f'Cannot subscribe {topic_name}: {e}')

    def generic_callback(self, msg):
        # print("Received message!")

def main(args=None):
    rclpy.init(args=args)
    node = CsvTopicSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

