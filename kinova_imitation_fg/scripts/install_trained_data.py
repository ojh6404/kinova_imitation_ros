#!/usr/bin/env python

from __future__ import print_function

import argparse
import multiprocessing

import jsk_data


def download_data(*args, **kwargs):
    p = multiprocessing.Process(
            target=jsk_data.download_data,
            args=args,
            kwargs=kwargs)
    p.start()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', dest='quiet', action='store_false')
    args = parser.parse_args()
    args.quiet

    PKG = 'fg_ros'

    download_data(
        pkg_name=PKG,
        path='trained_data/yolo7/2023-03-04-fg.pt',
        url='https://drive.google.com/uc?id=1gE_gibi6qJfOtK-5FTFjFK9YbGR4rw2P',
        md5='a71196ef07acb70d727a8a4bc947031f',
    )

    download_data(
        pkg_name=PKG,
        path='trained_data/yolo7/2023-03-06-fg-for-dspl.pt',
        url='https://drive.google.com/uc?id=19vs4hTGYC4IoAMkfOZfEPQKJZebQqkOY',
        md5='c86ccd71fddddf2f8d5303f16cd7e801',
    )

    # labels
    # ['apple', 'banana', 'cheez-it cracker box', 'domino sugar box', 'french’s mustard bottle', 'jell-o chocolate pudding box', 'jell-o strawberry gelatin box', 'lemon', 'master chef coffee can', 'orange', 'others', 'peach', 'pear', 'plum', 'pringles chips can', 'spam potted meat can', 'starkist tuna fish can', 'strawberry', 'tomato soup can']
    download_data(
        pkg_name=PKG,
        path='trained_data/yolo7/2023-03-07-fg-for-shelf.pt',
        url='https://drive.google.com/uc?id=1BAG25uOez6p5dWpDBJXTqUBZTTV0dn3',
        md5='58403e6ff7046b971086a846ad7f3d46',
    )


if __name__ == '__main__':
    main()
