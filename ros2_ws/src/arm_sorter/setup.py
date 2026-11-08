from setuptools import setup
from glob import glob
setup(name='arm_sorter', version='0.1.0', packages=['arm_sorter'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/arm_sorter']),
                  ('share/arm_sorter', ['package.xml']),
                  ('share/arm_sorter/config', glob('config/*'))],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': [
          'camera = arm_sorter.camera:main',
          'perception = arm_sorter.perception:main',
          'trajectory_bridge = arm_sorter.bridge:main',
          'sort_cycle = arm_sorter.sort_cycle:main']})
