import argparse
import os
import csv
import glob
import copy
import math
import numpy as np
import logging

def generate_line(pos1, pos2, speed, frame_delay = 0.1, start_frame = 0, color = [0, 0, 0], yaw = 0):
    dist = np.linalg.norm(pos1-pos2)
    delta = pos2 - pos1
    frames_count = math.ceil(dist/(speed*frame_delay))
    k = 1./frames_count
    frames = []
    for i in range(int(frames_count)+1):
        frame_pos = pos1 + k*i*delta
        frames.append({
            'number': start_frame + i,
            'x': frame_pos[0],
            'y': frame_pos[1],
            'z': frame_pos[2],
            'yaw': yaw,
            'red': color[0],
            'green': color[1],
            'blue': color[2],
        })
    return frames

def generate_positions(start_pos, nx, ny, dx, dy):
    len_x = (nx - 1)*dx
    len_y = (ny - 1)*dy
    x_center = start_pos[0]
    y_center = start_pos[1]
    z = start_pos[2]
    x0 = x_center - len_x/2.
    y0 = y_center + len_y/2.
    positions = []
    for iy in range(ny):
        for ix in range(nx):
            positions.append(np.array([x0+dx*ix, y0-dy*iy, z]))
    return positions

def parse_positions_file(filename):
    names = []
    pos = []
    try:
        pos_file = open(filename)
    except IOError:
        logging.error("File {} can't be opened".format(filename))
    else:
        with pos_file:
            n_str = pos_file.readline()[:-1].split(' ')
            nx = int(n_str[0])
            ny = int(n_str[1])
            dx = float(n_str[2])
            dy = float(n_str[3])
            pos_str = pos_file.readline()[:-1].split(' ')
            for i in range(3):
                pos.append(float(pos_str[i]))    
            for lines in pos_file:
                names.append(pos_file.readline()[:-1])
    return nx, ny, dx, dy, pos, names

def cut_to_closest_position(frames, pos, start_frame = -1):
    if start_frame == -1:
        return frames
    else:
        distances = np.zeros(len(frames)-start_frame)
        for i in range(start_frame, len(frames)):
            current_pos = np.array([frames[i]['x'], frames[i]['y'], frames[i]['z']])
            distances[i-start_frame] = np.linalg.norm(current_pos - pos)
        closest_index = distances.argmin()
        cut_frames = frames[:start_frame+closest_index]
        return cut_frames

def parse_animation_file(filename):
    imported_frames = []
    anim_id = ''
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
    return imported_frames, anim_id

def cut_file(filename, _from, _to, reverse = False):
    imported_frames = []
    anim_id = ""

    try:
        animation_file = open(filename)
    except IOError:
        logging.error("File {} can't be opened".format(filepath))
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

def change_landing(frames, land_pos, speed, start_frame = -1):
    cut_frames = cut_to_closest_position(frames, land_pos, start_frame)
    last_frame_pos = np.array([cut_frames[-1]['x'], cut_frames[-1]['y'], cut_frames[-1]['z']])
    line = generate_line(last_frame_pos, land_pos, speed, start_frame=len(cut_frames))
    return cut_frames + line

def save_frames(frames, animation_id, filename):
    csv_file = open(filename, mode='w+')
    with csv_file:
        csv_writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        if animation_id != "":
            csv_writer.writerow([animation_id])
        for frame in frames:
            csv_writer.writerow([frame['number'], frame['x'], frame['y'], frame['z'], frame['yaw'], frame['red'], frame['green'], frame['blue']])
    print("Successfully created file {}".format(path+'/'+os.path.basename(filename)))


if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description="Change landing positions")
    parser.add_argument('directory', nargs='?', default='.',
                        help="Directory with animation csv files. Default is '.'")
    parser.add_argument('-s','--start', type=int, default=-1,
                        help="Start count from this frame, default is -1 (from the end)")
    parser.add_argument('-p','--posfile', type=str, default="positions.txt",
                        help="Positions file. Default is positions.txt")
    parser.add_argument('-v','--speed', type=float, default=0.5,
                        help="Copter speed in m/s. Default is 0.5")
    args = parser.parse_args()

    start_frame = args.start
    speed = args.speed

    path = '{}/land_{}'.format(args.directory,start_frame)

    if not os.path.exists(path):
        try:
            os.mkdir(path)
        except OSError:
            print("Creation of the directory %s failed" % path)
        else:
            print("Successfully created the directory %s " % path)
    
    files = [f for f in glob.glob(args.directory + '/*.csv')]
    files.sort()

    nx, ny, dx, dy, pos, names = parse_positions_file(args.posfile)
    land_positions = generate_positions(pos, nx, ny, dx, dy)

    for i in range(len(files)):
        filename = os.path.basename(files[i])
        frames, animation_id = parse_animation_file(files[i])
        land_position = land_positions[i]
        new_frames = change_landing(frames, land_position, speed, start_frame)
        save_frames(new_frames, animation_id, path+'/'+filename)





