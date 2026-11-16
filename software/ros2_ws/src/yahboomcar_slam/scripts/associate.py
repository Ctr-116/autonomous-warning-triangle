#!/usr/bin/env python3
# TUM RGB-D timestamp association, preserving the original deterministic greedy
# algorithm. Python3 port of associate.py (Juergen Sturm, TUM, BSD).
import argparse
import sys


def read_file_list(filename):
    with open(filename, 'r') as handle:
        data = handle.read()
    lines = data.replace(",", " ").replace("\t", " ").split("\n")
    entries = [[v.strip() for v in line.split(" ") if v.strip() != ""]
               for line in lines if len(line) > 0 and line[0] != "#"]
    entries = [(float(l[0]), l[1:]) for l in entries if len(l) > 1]
    return dict(entries)


def associate(first_list, second_list, offset, max_difference):
    first_keys = set(first_list.keys())
    second_keys = set(second_list.keys())
    potential_matches = [(abs(a - (b + offset)), a, b)
                         for a in first_keys
                         for b in second_keys
                         if abs(a - (b + offset)) < max_difference]
    potential_matches.sort()
    matches = []
    for _diff, a, b in potential_matches:
        if a in first_keys and b in second_keys:
            first_keys.remove(a)
            second_keys.remove(b)
            matches.append((a, b))
    matches.sort()
    return matches


def main(argv=None):
    parser = argparse.ArgumentParser(description="Associate RGB and depth timestamps (TUM).")
    parser.add_argument("--first", default="rgb.txt", help="first timestamp file")
    parser.add_argument("--second", default="depth.txt", help="second timestamp file")
    parser.add_argument("--output", default="associations.txt", help="output associations file")
    parser.add_argument("--offset", type=float, default=0.0)
    parser.add_argument("--max-difference", type=float, default=0.02)
    args = parser.parse_args(argv)

    first_list = read_file_list(args.first)
    second_list = read_file_list(args.second)
    matches = associate(first_list, second_list, args.offset, args.max_difference)

    with open(args.output, mode="w") as out:
        for a, b in matches:
            a_data = " ".join(first_list[a])
            b_data = " ".join(second_list[b])
            print("%f %s %f %s" % (a, a_data, b - args.offset, b_data))
            out.write("%f %s %f %s\n" % (a, a_data, b - args.offset, b_data))
    return 0


if __name__ == '__main__':
    sys.exit(main())
