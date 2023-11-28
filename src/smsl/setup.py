from setuptools import setup, find_packages

setup(
    name='pysmsl',
    version='0.0.1',
    packages=find_packages(),
    install_requires=[
        'pyyaml',
        'networkx',
        'matplotlib',
    ],
    author='Yihao Liu',                 # Replace with your name
    author_email='yihao.jhu@gmail.com',     # Replace with your email
    description='*placeholder*',
    long_description=open('README.md').read(),  # Read the long description from a file
    long_description_content_type='text/markdown',  # Specify the type of long description
    url='https://github.com/SMSL-Project/pysmsl',  # Replace with the URL of your package repository
    license='MIT',                      # Choose an open-source license
)