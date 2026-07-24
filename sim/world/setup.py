import glob
from setuptools import setup

package_name = 'world'

# Some of data/obj/ is tracked and some is distributed out of band (see
# data/README.md), so which meshes exist depends on the checkout. glob() picks
# up whatever is actually there at build time instead of hardcoding a file list
# that would break the build when they are absent.
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    # ``target.scn.in`` is rendered into /tmp by core.launch.py when the
    # lightweight target scene is selected.  Install the template alongside
    # normal scenarios so this also works from an installed workspace.
    ('share/' + package_name + '/scenario',
     glob.glob('scenario/*.scn') + glob.glob('scenario/*.scn.in')),
    ('share/' + package_name + '/data/robot', glob.glob('data/robot/*.scn')),
    ('share/' + package_name + '/data/texture', glob.glob('data/texture/*.png')),
    ('share/' + package_name + '/data/obj',
     glob.glob('data/obj/*.obj') + glob.glob('data/obj/*.stl') + glob.glob('data/obj/*.mtl')),
    ('share/' + package_name, ['run.sh']),
]
# Drop any entry whose glob matched nothing — setuptools rejects a
# data_files entry with an empty file list.
data_files = [(dest, files) for dest, files in data_files if files]

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antoine',
    maintainer_email='antoine.esman7@gmail.com',
    description='Stonefish scenario assets: BlueROV2 model, environment meshes, .scn config',
    license='MIT',
)
