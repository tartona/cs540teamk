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
    parser.add_argument("--yellow_blocks", type=int, help="Number of yellow block to be generated in the world",
                        required=True)
    parser.add_argument("--wall_count", type=int, help="Number of random walls to be generated in the world",
                        required=False, default=0)
    parser.add_argument("--x_min", type=int, help="X-min value of drone world", default=-50, required=False)
    parser.add_argument("--x_max", type=int, help="X-max value of drone world", default=50, required=False)
    parser.add_argument("--y_max", type=int, help="Y-max value of drone world", default=50, required=False)
    parser.add_argument("--z_min", type=int, help="Z-min value of drone world", default=-50, required=False)
    parser.add_argument("--z_max", type=int, help="Z-max value of drone world", default=50, required=False)
    parser.add_argument("--filename", type=str, help="Configuration filename", required=True)

    return parser.parse_args()

if __name__ == "__main__":
    args = parse_args()

    total_blocks = args.red_blocks + args.blue_blocks + args.green_blocks + args.yellow_blocks
    if total_blocks == 0:
        print "Total blocks cannot be zero"
        exit(1)

    with open(args.filename, "wb") as csv_file:
        csv_writer = csv.writer(csv_file, delimiter=",")
        csv_writer.writerow(["0", "0", "0", "drone"])

        wall_locations = []
        walls = args.wall_count
        while walls > 0:
            x1 = random.randint(args.x_min, args.x_max)
            z1 = random.randint(args.z_min, args.z_max)
            x2 = random.randint(args.x_min, args.x_max)
            z2 = random.randint(args.x_min, args.x_max)

            if x1 == 0 and z1 == 0 or x2 == 0 and z1 == 0:
                continue

            if random.uniform(0,1) > 0.5:
                while x1 != x2:
                    for i in range(0, random.randint(0, args.y_max - 1)):
                        location = None
                        if x1 < x2:
                            location = x1, i, z1
                        else:
                            location = x2, i, z1

                        if location == (0, 0, 0):
                            break

                        if location not in wall_locations:
                            wall_locations.append(location)
                            if x1 < x2:
                                csv_writer.writerow([x1, i, z1, "black"])
                            else:
                                csv_writer.writerow([x2, i, z1, "black"])
                        else:
                            break
                    if x1 < x2:
                        x1 += 1
                    else:
                        x2 += 1
            else:
                while z1 != z2:
                    for i in range(0, random.randint(0, args.y_max - 1)):
                        location = None
                        if z1 < z2:
                            location = x1, i, z1
                        else:
                            location = x1, i, z2

                        if location == (0, 0, 0):
                            break

                        if location not in wall_locations:
                            wall_locations.append(location)
                            if z1 < z2:
                                csv_writer.writerow([x1, i, z1, "black"])
                            else:
                                csv_writer.writerow([x1, i, z2, "black"])
                        else:
                            break
                    if z1 < z2:
                        z1 += 1
                    else:
                        z2 += 1
            walls -= 1

        locations = []
        block_count = 0
        while block_count < total_blocks:
            x = random.randint(args.x_min, args.x_max)
            z = random.randint(args.z_min, args.z_max)
            if x == 0 and z == 0:
                continue
            else:
                for i in range(0, args.y_max - 1):
                    location = x, i, z
                    if location in locations or location in wall_locations:
                        continue
                    else:
                        locations.append(location)
                        block_count += 1
                        break

        red_count = 0
        blue_count = 0
        green_count = 0
        yellow_count = 0

        while (red_count + blue_count + green_count + yellow_count) != total_blocks:
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
            elif yellow_count < args.yellow_blocks:
                csv_writer.writerow([x, y, z, "yellow"])
                yellow_count += 1

    print "Drone world configuration filename: {}".format(args.filename)
    exit(0)
