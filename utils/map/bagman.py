"""

BagMan - Open Mower ROS Bag Manager

Even though this tool *should* back up your map file automatically, it's always a good idea to keep your own backup.

It's a console-based menu, so if you're really adventurous you could run this on the mower itself.

Requirements:
    Python 3.9 or later
    bagpy 0.5 or later

Example usage (see 'python3 bagman.py --help' for command line options):
    python3 bagman.py --input map.bag --output modified.bag

Features:
    - Automatic map backup
    - Set human-readable name for mowing and navigation areas (note that this is only currently visible in bagman)
    - Remove individual areas
    - Change the order of the areas (this sets the order in which areas are mowed)

Changelog:
    Version 0.0.1 - 2023-08-16 - Initial release

TODO:
    - Figure out if there's a way to selectively disable areas. Changing the topic doesn't work. Changing the package
      for the message does work, but any disabled areas are removed if you add a new area in Open Mower.
      Ideas are welcome.
    - Determine if there's a maximum length for area names (and enforce the limit if so)

"""


import argparse
import datetime
import hashlib
import logging
import os.path
import time
import zipfile
from logging.handlers import RotatingFileHandler
from typing import Dict
from typing import List
from typing import Optional

try:
    import rosbag
    import rospy
except ImportError:
    raise ImportError("You need to install the 'bagpy' package before running this (e.g. pip install bagpy>=0.5)")


class BagMan:
    TOPIC_MOWING_AREAS = "mowing_areas"
    TOPIC_NAVIGATION_AREAS = "navigation_areas"
    ALL_TOPICS_THAT_CAN_BE_NAMED = [TOPIC_MOWING_AREAS, TOPIC_NAVIGATION_AREAS]

    def __init__(self, console_log_level: int = logging.INFO, file_log_level=logging.DEBUG):
        self.log: logging.Logger = logging.getLogger("bagman")
        self.log.setLevel(logging.DEBUG)
        if not self.log.hasHandlers():
            log_format = logging.Formatter(
                fmt="%(asctime)s %(levelname)s in %(module)s: %(message)s",
                datefmt="[%Y-%m-%d %H:%M:%S]"
            )
            console_handler = logging.StreamHandler()
            console_handler.setFormatter(log_format)
            console_handler.setLevel(console_log_level)
            self.log.addHandler(console_handler)
            log_dir = os.path.abspath("logs")
            new_log_path = os.path.join(log_dir, "bagman.log")
            existing_log_file: bool = os.path.exists(new_log_path)
            if not os.path.exists(log_dir):
                os.makedirs(log_dir)
            logfile = RotatingFileHandler(filename=new_log_path, backupCount=50)
            logfile.setFormatter(log_format)
            logfile.setLevel(file_log_level)
            if existing_log_file:
                logfile.doRollover()
            self.log.addHandler(logfile)
        self.log.info("Starting...")
        self.__input_file_path: Optional[str] = None
        self.__output_file_path: Optional[str] = None
        self.__dirty: bool = False

    def _present_menu(
            self,
            title: str,
            choices: Dict[str, str],
            ignore_case: bool = True,
            subtitle: Optional[str] = None,
    ) -> str:
        """
        Displays a menu and prompts the user for their selection
        :param choices: Dictionary where the key is the choice number/name, and the value is the description.
        :return: Key for the chosen item
        """
        while True:
            self.log.info("")
            self.log.info(f"===== {title} =====")
            if isinstance(subtitle, str):
                self.log.info(f"===== {subtitle} =====")
            max_key_len: int = sorted([len(str(k)) for k in choices.keys()])[-1]
            for key, description in choices.items():
                padding = " " * (max_key_len - len(str(key)))
                self.log.info(f"[{key}] {padding} {description}")
            time.sleep(0.3)
            choice: str = input("> ")
            self.log.debug("User entered '{}'".format(choice))
            if choice in choices or (ignore_case and choice.lower() in [c.lower() for c in choices]):
                return choice
            else:
                self.log.error(f"Invalid choice '{choice}'")
                time.sleep(1)

    def hash_bag_file(self, file_path: str) -> str:
        """
        :param file_path:
        :return: SHA1 hash of the file
        """
        self.log.debug(f"Hashing {file_path}")
        file_hash = hashlib.sha1()
        with open(file_path, "rb") as downloaded_file:
            while True:
                chunk = downloaded_file.read(1024 ** 2)
                if not chunk:
                    break
                file_hash.update(chunk)
        hash_result: str = file_hash.hexdigest()
        self.log.debug(f"Resulting hash: {hash_result}")
        return hash_result

    def backup_bag(self, file_path: str):
        """
        Creates a backup of the given file (if necessary).
        File naming convention is <original_filename>_<utc_datetime>_<uncompressed_hash>.zip
        :param file_path:
        :return:
        """
        # Create a directory for backups if it doesn't exist
        backups_dir = os.path.abspath("backups")
        if not os.path.exists(backups_dir):
            self.log.info(f"Creating backup directory {backups_dir}")
            os.makedirs(backups_dir)

        # Hash the file and ensure this version isn't already backed up
        file_hash: str = self.hash_bag_file(file_path=file_path)
        existing_hashes: List[str] = [
            os.path.splitext(file_name)[0].split("_")[-1]  # See file naming convention in docstring above
            for file_name in os.listdir(backups_dir)
        ]
        if file_hash in existing_hashes:
            self.log.info(f"This version of {file_path} has already been backed up (hash {file_hash})")
            return

        # Make the backup
        backup_file_path: str = os.path.join(
            backups_dir,
            "_".join([
                os.path.basename(file_path),
                datetime.datetime.utcnow().strftime("%Y-%m-%dT%H%M%S"),
                f"{file_hash}.zip"
            ]),
        )
        self.log.info(f"Backing up {file_path} (hash {file_hash}) to {backup_file_path}")
        with zipfile.ZipFile(backup_file_path, 'w', zipfile.ZIP_DEFLATED, compresslevel=9) as zip_output:
            zip_output.write(filename=file_path, arcname=os.path.basename(file_path))

        old_backups_to_remove: List[str] = sorted(os.listdir(backups_dir), reverse=True)[30:]
        for old_backup in old_backups_to_remove:
            self.log.info(f"Pruning old backup {old_backup}")
            os.remove(old_backup)

    def read_bag(self, file_path: str) -> List[rosbag.bag.BagMessage]:
        """
        Reads the given bag file into memory. Note that we do this so we can
        :param file_path:
        :return:
        """
        self.log.info(f"Loading {file_path}")

        if not os.path.exists(file_path):
            raise OSError(f"File '{file_path}' does not exist!")

        with rosbag.Bag('map.bag', 'r') as bag:
            return [m for m in bag.read_messages()]

    def save_bag(self, file_path: str, items: List[rosbag.bag.BagMessage]):
        """
        :param file_path:
        :param items:
        :return:
        """
        self.log.info(f"Saving to {file_path}")

        while True:
            if os.path.exists(file_path):
                choice_yes = "yes"
                choice_no = "no"
                choice_change = "change"
                overwrite_choice: str = self._present_menu(
                    title=f"Are you sure you want to overwrite {file_path}?",
                    choices={
                        choice_yes: "Yes, overwrite the file",
                        choice_no: "No, discard changes",
                        choice_change: "Change output file path"
                    }
                )
                if overwrite_choice == choice_yes:
                    self.log.warning(f"User chose to overwrite existing file {file_path}!")
                    break
                elif overwrite_choice == choice_no:
                    self.log.warning("User chose to discard changes!")
                    return
                elif overwrite_choice == choice_change:
                    self.log.info("Enter new output path:")
                    time.sleep(0.3)
                    file_path = os.path.abspath(input("> "))
                    self.log.info("User chose to change output to {}".format(file_path))
                    continue

        with rosbag.Bag(file_path, "w") as out_bag:
            for item in items:
                out_bag.write(item.topic, item.message, item.timestamp)

        self.__dirty = False

    def parse_command_line_args(self):
        self.log.debug("Parsing command line args...")
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "--input", "-i",
            required=True,
            action="store",
            help="Path of file to read. Normally this is map.bag"
        )
        parser.add_argument(
            "--output", "-o",
            required=True,
            action="store",
            help="Where to save the output. This can be the same as your input if you want"
        )

        args = parser.parse_args()
        self.__input_file_path: str = os.path.abspath(args.input)
        self.__output_file_path: str = os.path.abspath(args.output)

    @classmethod
    def _clone_timestamp(cls, timestamp: rospy.Time) -> rospy.Time:
        return rospy.Time(
            secs=timestamp.secs,
            nsecs=timestamp.nsecs
        )

    @classmethod
    def _stringify_bag_item(
            cls,
            item: rosbag.bag.BagMessage,
            pad_topic_col_to_width: int = 0,
            timestamp_col_padding: int = 0,
    ) -> str:
        converted_timestamp: datetime.datetime = datetime.datetime.fromtimestamp(
            item.timestamp.secs + item.timestamp.nsecs / 1_000_000_000
        )
        parts = [
            f"Topic: '{item.topic}'",
            " " * max(pad_topic_col_to_width - len(item.topic), 0) if pad_topic_col_to_width else "",
            f"Timestamp: {converted_timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')}",
            " " * max(timestamp_col_padding, 0) if timestamp_col_padding else "",
            # Note that some messages don't contain the name attribute (e.g. docking_point)
            f"Name: '{item.message.name}'" if isinstance(getattr(item.message, "name", None), str) else "",
        ]
        return " ".join([part for part in parts if part])

    def interactive_menu(self, items: List[rosbag.bag.BagMessage]):
        choice_save = "save"
        choice_quit = "quit"
        # Main menu
        while True:
            max_topic_width: int = sorted([len(item.topic) for item in items])[-1]
            base_menu_choice: str = self._present_menu(
                title="Please select an item or operation to continue",
                choices={
                    k: v for k, v in {
                        # List of items in the bag
                        **{
                            str(idx): self._stringify_bag_item(
                                item,
                                pad_topic_col_to_width=max_topic_width + 1,
                                timestamp_col_padding=3,
                            )
                            for idx, item in enumerate(items)
                        },
                        # Operations
                        choice_save: f"Save to {self.__output_file_path}" if self.__dirty else "",
                        choice_quit: "Quit",
                    }.items() if v
                }
            )

            # Handle save/quit commands
            if base_menu_choice == choice_save:
                self.log.info("User chose to save")
                self.save_bag(file_path=self.__output_file_path, items=items)
                continue
            elif base_menu_choice == choice_quit:
                self.log.info("User chose to quit")
                if self.__dirty:
                    quit_menu_choice: str = self._present_menu(
                        title="You have unsaved changes!",
                        choices={
                            choice_save: f"Save to {self.__output_file_path}",
                            choice_quit: "Quit without saving",
                        }
                    )
                    if quit_menu_choice == choice_save:
                        self.log.info("User chose to save")
                        self.save_bag(file_path=self.__output_file_path, items=items)
                    elif quit_menu_choice == choice_quit:
                        self.log.warning("User chose to discard changes")
                    else:
                        continue
                return

            # Submenu for operating on individual items
            item_idx: int = int(base_menu_choice)
            self.log.debug(f"Item {item_idx} selected")
            while True:
                selected_item: rosbag.bag.BagMessage = items[item_idx]
                is_first: bool = item_idx == 0
                is_last: bool = item_idx == len(items) - 1
                is_nameable: bool = selected_item.topic in self.ALL_TOPICS_THAT_CAN_BE_NAMED
                choice_set_name = "name"
                choice_remove = "remove"
                choice_move_to_first_pos = "first"
                choice_move_to_last_pos = "last"
                choice_move_up = "up"
                choice_move_down = "down"
                choice_back = "back"
                item_menu_choice: str = self._present_menu(
                    title=f"Please select an operation to perform on {self._stringify_bag_item(selected_item)}",
                    subtitle="Note that enable/disable will change topic and position changes will change timestamp",
                    choices={
                        k: v for k, v in {
                            # Note that not all choices will be available for all items. We set the value to a
                            # blank string for any choices we want to filter out
                            choice_set_name: "Set name" if is_nameable else "",
                            choice_move_to_first_pos: "Move to first position" if not is_first else "",
                            choice_move_to_last_pos: "Move to last position" if not is_last else "",
                            choice_move_up: "Move up one position" if not is_first else "",
                            choice_move_down: "Move down one position" if not is_last else "",
                            choice_remove: "Remove from bag",
                            choice_back: "Go back to the main menu",
                        }.items() if v
                    }
                )

                if item_menu_choice == choice_back:
                    self.log.debug("Going back to the main menu")
                    break

                if item_menu_choice == choice_set_name:
                    self.log.info("Enter new name (or press enter to go back)")
                    time.sleep(0.3)
                    new_name: str = input("> ")
                    if new_name:
                        self.log.info(f"Setting name to '{new_name}'")
                        items[item_idx].message.name = new_name
                        self.__dirty = True
                    else:
                        self.log.debug("User chose to go back to the item menu")
                elif item_menu_choice == choice_remove:
                    self.log.info("Removing item...")
                    items.pop(item_idx)
                    self.__dirty = True
                    break
                elif item_menu_choice == choice_move_to_first_pos:
                    self.log.info("Moving item to first position...")
                    selected_item.timestamp.secs = items[0].timestamp.secs - 60
                    items.pop(item_idx)
                    items.insert(0, selected_item)
                    item_idx = 0
                    self.__dirty = True
                elif item_menu_choice == choice_move_to_last_pos:
                    self.log.info("Moving item to last position...")
                    selected_item.timestamp.secs = items[-1].timestamp.secs + 60
                    items.pop(item_idx)
                    items.append(selected_item)
                    item_idx = len(items) - 1
                    self.__dirty = True
                elif item_menu_choice == choice_move_up:
                    self.log.info("Moving item up one position...")
                    old_timestamp = self._clone_timestamp(selected_item.timestamp)
                    # Note that we can't just swap the entire timestamp, however we can modify components one by one
                    selected_item.timestamp.secs = items[item_idx - 1].timestamp.secs
                    selected_item.timestamp.nsecs = items[item_idx - 1].timestamp.nsecs
                    items[item_idx - 1].timestamp.secs = old_timestamp.secs
                    items[item_idx - 1].timestamp.nsecs = old_timestamp.nsecs
                    items.pop(item_idx)
                    items.insert(item_idx - 1, selected_item)
                    item_idx -= 1
                    self.__dirty = True
                elif item_menu_choice == choice_move_down:
                    self.log.info("Moving item down one position...")
                    old_timestamp = self._clone_timestamp(selected_item.timestamp)
                    # Note that we can't just swap the entire timestamp, however we can modify components one by one
                    selected_item.timestamp.secs = items[item_idx + 1].timestamp.secs
                    selected_item.timestamp.nsecs = items[item_idx + 1].timestamp.nsecs
                    items[item_idx + 1].timestamp.secs = old_timestamp.secs
                    items[item_idx + 1].timestamp.nsecs = old_timestamp.nsecs
                    items.pop(item_idx)
                    items.insert(item_idx + 1, selected_item)
                    item_idx += 1
                    self.__dirty = True

    def run(self):
        self.parse_command_line_args()
        self.backup_bag(file_path=self.__input_file_path)
        self.interactive_menu(items=self.read_bag(file_path=self.__input_file_path))
        self.log.info("Done!")


if __name__ == "__main__":
    BagMan(console_log_level=logging.DEBUG).run()
