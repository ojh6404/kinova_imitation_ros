#!/usr/bin/env python

from jsk_data import download_data


def main():
    # PKG = 'jsk_robocup_perception'
    PKG = 'kinova_imitation_perception'

    download_data(
        pkg_name=PKG,
        path='sample/data/2017-06-20-12-00-00_people_images_in_lab.bag.tgz',
        url='https://drive.google.com/uc?id=1VYLgjccB9sCa5ht32r3FLjUC2fhFkyZI',
        md5='0a397a2ff14c07954cc0b9178e33600d',
        extract=True,
    )


if __name__ == '__main__':
    main()
