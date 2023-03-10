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

    # PKG = 'jsk_robocup_perception'
    PKG = 'kinova_imitation_perception'

    download_data(
        pkg_name=PKG,
        path='trained_data/E2Pose/resources_post.tar.gz',
        url='https://s3.ap-northeast-2.wasabisys.com/pinto-model-zoo/333_E2Pose/resources_post.tar.gz',
        extract=True,
        md5='e1ed360f10077d2c2c333cb298efab3b',
    )

    download_data(
        pkg_name=PKG,
        path='trained_data/clip/2023-03-05.pth',
        url='https://drive.google.com/uc?id=1bRLxAPJ70E8WmpemxlNw-ZJktSrHgqbx',
        extract=False,
        md5='d7a008818fd88e150602f8e93a50f8e1',
    )


if __name__ == '__main__':
    main()
