from vmbpy import *

import xml.etree.ElementTree as ET

import argparse
import os   
from typing import Optional

def abort(msg):
    print(f'{msg}')
    exit(1)

def extract_camera_id_from_xml(file_path):
    try:
        # Parse the XML (CML) file
        tree = ET.parse(file_path)
        root = tree.getroot()

        # Find the 'CameraInfo' element and extract its 'Id' attribute
        camera_info = root.find('CameraInfo')
        if camera_info is not None and 'Id' in camera_info.attrib:
            camera_id = camera_info.attrib['Id']
            print(f"Camera Info Id: {camera_id}")
            return camera_id
        else:
            print("No CameraInfo or Id attribute found in the file.")

    except ET.ParseError as e:
        print(f"Error parsing CML file: {e}")
    except FileNotFoundError:
        print(f"File not found: {file_path}")

       
def main():
    parser = argparse.ArgumentParser(description='Load camera configuration')
    parser.add_argument('--settings_folder', type=str, default='/path/to/settings/folder', help='Folder to save/load camera settings')
    args = parser.parse_args()

    # Check if the settings folder exists.
    if not os.path.exists(args.settings_folder):
        abort('Given settings folder does not exist')

    settings = {}
    for file in os.listdir(args.settings_folder):
        if file.endswith('.xml'):
            print(f'Loading settings from {file}')
            settings_file = os.path.join(args.settings_folder, file)
            camera_id = extract_camera_id_from_xml(settings_file)
            settings[camera_id] = settings_file

    with VmbSystem.get_instance() as vmb:
        for cam in vmb.get_all_cameras():
            with cam:
                # Restore settings to initial value.
                try:
                    cam.UserSetSelector.set('Default')

                except (AttributeError, VmbFeatureError):
                    print('Failed to set Feature \'UserSetSelector\'')

                try:
                    cam.UserSetLoad.run()
                    print("--> All feature values have been restored to default")

                except (AttributeError, VmbFeatureError):
                    abort('Failed to run Feature \'UserSetLoad\'')

                cam_id = cam.get_id()

                if cam_id in settings.keys():
                    print(f'Loading settings for camera {cam_id}')
                    cam.load_settings(settings[cam_id], PersistType.All)
                else:
                    print(f'No settings found for camera {cam_id}')

if __name__ == '__main__':
    main()