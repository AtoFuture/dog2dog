"""Extract map-frame RETURN feedback from a finalized bag for the C++ replay tool."""
import argparse
import csv
import json
from pathlib import Path
import sqlite3
from rclpy.serialization import deserialize_message
from nav2_msgs.action import NavigateToPose


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path)
    args=p.parse_args();root=args.directory.resolve()
    report=json.loads((root/'report.json').read_text())
    if report.get('recorder_exit_code')!=0 or not (root/'bag/metadata.yaml').is_file():
        p.error('Use a finalized recording')
    goals={e['goal_id'] for e in report['return_events'] if e['status']=='accepted'}
    db=sqlite3.connect('file:'+str(next((root/'bag').glob('*.db3')))+'?mode=ro',uri=True)
    topic_id=db.execute('SELECT id FROM topics WHERE name=?',('/robot1/navigate_to_pose/_action/feedback',)).fetchone()[0]
    output=root/'return_map_progress.csv'
    with output.open('w') as stream:
        writer=csv.writer(stream);writer.writerow(['sim_s','map_x','map_y'])
        # Sample every 50 ms of bag receipt time before decoding high-rate feedback.
        query='SELECT data FROM messages WHERE id IN (SELECT MAX(id) FROM messages WHERE topic_id=? GROUP BY timestamp/50000000) ORDER BY timestamp'
        for data, in db.execute(query,(topic_id,)):
            msg=deserialize_message(data,NavigateToPose.Impl.FeedbackMessage)
            if bytes(msg.goal_id.uuid).hex() not in goals:continue
            pose=msg.feedback.current_pose
            if pose.header.frame_id!='map':raise ValueError('Expected map-frame feedback')
            stamp=pose.header.stamp.sec+pose.header.stamp.nanosec*1e-9
            writer.writerow([stamp,pose.pose.position.x,pose.pose.position.y])
    db.close();print(output)


if __name__=='__main__':main()
