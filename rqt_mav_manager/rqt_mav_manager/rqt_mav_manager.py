#!/usr/bin/env python3

import sys

from rqt_gui.main import Main
  
def main():
# Run the plugin
  main = Main()
  sys.exit(main.main(sys.argv, standalone="rqt_mav_manager.MavManagerUi"))

if __name__ == "__main__":
  main()
