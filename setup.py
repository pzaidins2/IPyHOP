from setuptools import setup

setup(
    name='IPyHOPPER',
    version='0.1.0',
    packages=[ 'ipyhop' ],
    url='https://github.com/pzaidins2/IPyHOPPER.git',
    license='',
    author='paulz',
    author_email='pzaidins@umd.edu',
    description='IPyHOPPER HTN Planner',
    install_requires=[
        "numpy",
        "networkx",
        "matplotlib"
    ]
)
