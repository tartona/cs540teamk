import argparse
import random
import csv

def parse_args():
    parser = argparse.ArgumentParser(description="Generate a random configuration file for drone world.")
    parser.add_argument("--red_blocks", type=int, help="Number of reds block to be generated in the world",
                        required=True)
    parser.add_argument("--blue_blocks", type=int, help="Number of reds block to be generated in the world",
                        required=True)
    parser.add_argument("--green_blocks", type=int, help="Number of reds block to be generated in the world",
                        required=True)
    parser.add_argument("--y_limit", type=int, help="Limit the y location (height) of the blocks", default=1,
                        required=False)
    parser.add_argument("--x_min", type=int, help="X-min value of drone world", default=-50, required=False)
    parser.add_argument("--x_max", type=int, help="X-max value of drone world", default=50, required=False)
    parser.add_argument("--y_min", type=int, help="Y-min value of drone world", default=0, required=False)
    parser.add_argument("--y_max", type=int, help="Y-max value of drone world", default=50, required=False)
    parser.add_argument("--z_min", type=int, help="Z-min value of drone world", default=-50, required=False)
    parser.add_argument("--z_max", type=int, help="Z-max value of drone world", default=50, required=False)
    parser.add_argument("--filename", type=str, help="Configuration filename", required=True)

    return parser.parse_args()

if __name__ == "__main__":
    args = parse_args()

    total_blocks = args.red_blocks + args.blue_blocks + args.green_blocks
    if total_blocks == 0:
        print "Total blocks cannot be zero"
        exit(1)

    locations = []
    block_count = 0
    while block_count < total_blocks:
        x = random.randint(args.x_min, args.x_max)
        y = random.randint(args.y_min, min(args.y_max, args.y_limit - 1))
        z = random.randint(args.z_min, args.z_max)
        location = x, y, z
        if location == (0, 0, 0) or location in locations:
            continue
        else:
            locations.append(location)
            block_count += 1

    red_count = 0
    blue_count = 0
    green_count = 0
    with open(args.filename, "wb") as csv_file:
        csv_writer = csv.writer(csv_file, delimiter=",")
        csv_writer.writerow(["0", "0", "0", "drone"])
        while (red_count + blue_count + green_count) != total_blocks:
            x, y, z = locations.pop(0)
            if red_count < args.red_blocks:
                csv_writer.writerow([x, y, z, "red"])
                red_count += 1
            elif blue_count < args.blue_blocks:
                csv_writer.writerow([x, y, z, "blue"])
                blue_count += 1
            elif green_count < args.green_blocks:
                csv_writer.writerow([x, y, z, "green"])
                green_count += 1

    print "Drone world configuration filename: {}".format(args.filename)
    exit(0)
