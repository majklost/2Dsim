import numpy as np
from setuptools import setup
from Cython.Build import cythonize


setup(
    name='2DSim',
    version='',
    ext_modules=cythonize(["./deform_plan/rrt_utils/dist.pyx"]),
    include_dirs=[np.get_include()],
    packages=['deform_plan', 'deform_plan.assets', 'deform_plan.assets.PM', 'deform_plan.helpers',
              'deform_plan.messages', 'deform_plan.samplers', 'deform_plan.rrt_utils'],
    package_dir={'': ''},
    url='',
    license='',
    author='michal',
    author_email='',
    description=''
)
