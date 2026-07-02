import glob
from setuptools import setup

package_name = 'world'

# Mesh files (data/obj/*.obj, *.mtl) are gitignored (~313 MB — see
# data/README.md) but may or may not be present on disk depending on the
# checkout. glob() picks up whatever's actually there at build time instead
# of hardcoding a file list that would break the build when they're absent.
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/scnenario', glob.glob('scnenario/*.scn')),
    ('share/' + package_name + '/data/robot', glob.glob('data/robot/*.scn')),
    ('share/' + package_name + '/data/texture', glob.glob('data/texture/*.png')),
    ('share/' + package_name + '/data/obj', glob.glob('data/obj/*.obj') + glob.glob('data/obj/*.mtl')),
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
