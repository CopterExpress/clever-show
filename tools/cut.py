import argparse
import os
import csv
import glob
import copy
import logging

def cut_file(filename, _from, _to, reverse = False):
    imported_frames = []
    anim_id = ""

    try:
        animation_file = open(filename)
    except IOError:
        logging.error("File {} can't be opened".format(filename))
    else:
        with animation_file:
            csv_reader = csv.reader(
                animation_file, delimiter=',', quotechar='|'
            )
            row_0 = csv_reader.next()
            if len(row_0) == 1:
                anim_id = row_0[0]
                logging.debug("Got animation_id: {}".format(anim_id))
            else:
                logging.debug("No animation id in file")
                frame_number, x, y, z, yaw, red, green, blue = row_0
                imported_frames.append({
                    'number': int(frame_number),
                    'x': float(x),
                    'y': float(y),
                    'z': float(z),
                    'yaw': float(yaw),
                    'red': int(red),
                    'green': int(green),
                    'blue': int(blue),
                })
            for row in csv_reader:
                frame_number, x, y, z, yaw, red, green, blue = row
                imported_frames.append({
                    'number': int(frame_number),
                    'x': float(x),
                    'y': float(y),
                    'z': float(z),
                    'yaw': float(yaw),
                    'red': int(red),
                    'green': int(green),
                    'blue': int(blue),
                })

        if _to == 0 or _to >= len(imported_frames):
            _to = len(imported_frames)-1

        path = '{}/cut_{}_{}'.format(os.path.dirname(filename),_from,_to)

        if reverse:
            path += '_r'
        
        csv_file = open(path+'/'+os.path.basename(filename), mode='w+')
        with csv_file:
            csv_writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            if anim_id != "":
                csv_writer.writerow([anim_id])
            if not reverse:
                for i in range(_from, _to+1):
                    csv_writer.writerow([imported_frames[i]['number'], imported_frames[i]['x'], imported_frames[i]['y'], imported_frames[i]['z'],
                                        imported_frames[i]['yaw'], imported_frames[i]['red'], imported_frames[i]['green'], imported_frames[i]['blue']])
            else:
                for i in range(_from):
                    csv_writer.writerow([imported_frames[i]['number'], imported_frames[i]['x'], imported_frames[i]['y'], imported_frames[i]['z'],
                                        imported_frames[i]['yaw'], imported_frames[i]['red'], imported_frames[i]['green'], imported_frames[i]['blue']])
                for i in range(_to, len(imported_frames)):
                    csv_writer.writerow([imported_frames[i]['number'], imported_frames[i]['x'], imported_frames[i]['y'], imported_frames[i]['z'],
                                        imported_frames[i]['yaw'], imported_frames[i]['red'], imported_frames[i]['green'], imported_frames[i]['blue']])


        print("Successfully created file {}".format(path+'/'+os.path.basename(filename)))

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description="cut animation")
    parser.add_argument('directory', nargs='?', default='.',
                        help="Directory with animation csv files. Default is '.'")
    parser.add_argument('-f','--frm', type=int, default=0,
                        help="Cut from this frame, default is 0 (from the beginning)")
    parser.add_argument('-t','--to', type=int, default=0,
                        help="Cut to this frame (including this one), default is 0 (to the end)")  
    parser.add_argument('-r', '--reverse', action='store_true', help="Reverse cutting, this tool will cut from the beginning to <frm> frame and from <to> frame to the end")
    args = parser.parse_args()

    _from = args.frm
    _to = args.to

    path = '{}/cut_{}_{}'.format(args.directory,_from,_to)
    if args.reverse:
        path += '_r'

    if not os.path.exists(path):
        try:
            os.mkdir(path)
        except OSError:
            print("Creation of the directory %s failed" % path)
        else:
            print("Successfully created the directory %s " % path)
    
    files = [f for f in glob.glob(args.directory + '/*.csv')]
    for f in files:
        cut_file(f, _from, _to, args.reverse)




