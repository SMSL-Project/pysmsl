from setuptools import setup, find_packages

setup(
    name='pysmsl',
    version='0.1.1',
    packages=find_packages(exclude=['smsl.smsl_parser', 'smsl.smsl_parser.*']),
    install_requires=[
        'pyyaml',
        'networkx',
        'matplotlib',
    ],
    author='Yihao Liu',
    author_email='yihao.jhu@gmail.com',
    description='State Machine Serialization Language',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/SMSL-Project/pysmsl',
    license='MIT',
)