#!/usr/bin/env python3

import subprocess
import os

def run_test(image_path: str, chunk_size: int = 0):
    cmd = [
        'ros2', 'run', 'image_transfer', 'image_transfer_client',
        '--ros-args',
        '-p',
        f'image_path:={image_path}',
        '-p',
        'test_mode:=true',
        '-p',
        f'chunk_size:={chunk_size}',
    ]

    try:
        subprocess.run(cmd, check=True)
        return True
    except:
        return False

def main():

    base_dir = "test_images"

    for root, dirs, files in os.walk(base_dir):
        for file in files:
            abs_path = os.path.join(root, file)

            rel_path = os.path.relpath(abs_path, base_dir)

            full_rel_path = os.path.normpath(os.path.join(base_dir, rel_path))
           
            
            run_test(full_rel_path)
            run_test(full_rel_path, 1000000)

            


if __name__ == "__main__":
    main()
