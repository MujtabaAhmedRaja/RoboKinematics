from setuptools import setup, find_packages

setup(
    name="robokinematics",
    version="1.0.0",
    description="A standalone Python 3D Kinematics mathematical engine and Hardware Abstraction Layer for robotic arms.",
    author="RoboKinematics Pro",
    packages=find_packages(),
    py_modules=["kinematics"],
    install_requires=[
        "numpy",
        "scipy"
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
    ],
)
