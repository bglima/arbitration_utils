# arbitration_utils

### How to build

Before building, you need to clone the following packages:

```bash
# Clone the objects loader packages
git clone https://github.com/CNR-STIIMA-IRAS/object_loader.git
```

Now, clone the non-ROS FuzzyLite package:

```bash
# Clone the repo into the main branch into a folder of your preference.
# It does not need to be inside your catkin_ws since it is not a ROS package
git clone https://github.com/fuzzylite/fuzzylite

# Enter the cloned folder
cd fuzzylite

# Compile the package
make

# Install into your system
sudo make install

# Export the installation local of your library (maybe into your .bashrc)
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

# Now, you should be able to compile this package without problems.
catkin build arbitration_utils
```
