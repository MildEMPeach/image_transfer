#!/usr/bin/env python3

import subprocess
import os

def run_test(image_path: str, compression_type: str):
    cmd = [
        'ros2', 'run', 'image_transfer', 'image_transfer_client',
        '--ros-args',
        '-p',
        f'image_path:={image_path}',
        '-p',
        'test_mode:=true',
        '-p',
        f'compression_type:={compression_type}',
    ]

    try:
        subprocess.run(cmd, check=True)
        return True
    except:
        return False

def main():
    # run_test("test_images/basic/blue_640x480.png", "jpeg")

    base_dir = "test_images"

    for root, dirs, files in os.walk(base_dir):
        for file in files:
            abs_path = os.path.join(root, file)

            rel_path = os.path.relpath(abs_path, base_dir)

            full_rel_path = os.path.normpath(os.path.join(base_dir, rel_path))
           
            run_test(full_rel_path, "jpeg")
            run_test(full_rel_path, "none")

            


if __name__ == "__main__":
    main()
