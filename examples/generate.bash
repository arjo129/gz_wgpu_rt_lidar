#!/bin/bash


# 900 cars, 640 lidars

# RTX
python3 world_gen.py --geometry car --row 30 --col 30 --ground 1000 --sensor rt --lidars 1 --samples 640 > rt_1_640_900.sample.sdf

python3 world_gen.py --geometry car --row 30 --col 30 --ground 1000 --sensor rt --lidars 4 --samples 640 > rt_4_640_900.sample.sdf

python3 world_gen.py --geometry car --row 30 --col 30 --ground 1000 --sensor rt --lidars 10 --samples 640 > rt_10_640_900.sample.sdf

python3 world_gen.py --geometry car --row 30 --col 30 --ground 1000 --sensor rt --lidars 20 --samples 640 > rt_20_640_900.sample.sdf


# gz
python3 world_gen.py --geometry car --row 30 --col 30 --ground 1000 --sensor gazebo --lidars 1 --samples 640 > gz_1_640_900.sample.sdf

python3 world_gen.py --geometry car --row 30 --col 30 --ground 1000 --sensor gazebo --lidars 4 --samples 640 > gz_4_640_900.sample.sdf

python3 world_gen.py --geometry car --row 30 --col 30 --ground 1000 --sensor gazebo --lidars 10 --samples 640 > gz_10_640_900.sample.sdf

# 100 cars, 640 lidars

# RTX
python3 world_gen.py --geometry car --row 10 --col 10 --ground 1000 --sensor rt --lidars 1 --samples 640 > rt_1_640_100.sample.sdf

python3 world_gen.py --geometry car --row 10 --col 10 --ground 1000 --sensor rt --lidars 4 --samples 640 > rt_4_640_100.sample.sdf

python3 world_gen.py --geometry car --row 10 --col 10 --ground 1000 --sensor rt --lidars 10 --samples 640 > rt_10_640_100.sample.sdf

python3 world_gen.py --geometry car --row 10 --col 10 --ground 1000 --sensor rt --lidars 20 --samples 640 > rt_20_640_100.sample.sdf


# gz
python3 world_gen.py --geometry car --row 10 --col 10 --ground 1000 --sensor gazebo --lidars 1 --samples 640 > gz_1_640_100.sample.sdf

python3 world_gen.py --geometry car --row 10 --col 10 --ground 1000 --sensor gazebo --lidars 4 --samples 640 > gz_4_640_100.sample.sdf

python3 world_gen.py --geometry car --row 10 --col 10 --ground 1000 --sensor gazebo --lidars 10 --samples 640 > gz_10_640_100.sample.sdf


# 100 cars, 6400 lidars

# RTX
python3 world_gen.py --geometry car --row 10 --col 10 --ground 1000 --sensor rt --lidars 1 --samples 6400 > rt_1_6400_100.sample.sdf

python3 world_gen.py --geometry car --row 10 --col 10 --ground 1000 --sensor rt --lidars 4 --samples 6400 > rt_4_6400_100.sample.sdf

python3 world_gen.py --geometry car --row 10 --col 10 --ground 1000 --sensor rt --lidars 10 --samples 6400 > rt_10_6400_100.sample.sdf

python3 world_gen.py --geometry car --row 10 --col 10 --ground 1000 --sensor rt --lidars 20 --samples 6400 > rt_20_6400_100.sample.sdf


# gz
python3 world_gen.py --geometry car --row 10 --col 10 --ground 1000 --sensor gazebo --lidars 1 --samples 6400 > gz_1_6400_100.sample.sdf

python3 world_gen.py --geometry car --row 10 --col 10 --ground 1000 --sensor gazebo --lidars 4 --samples 6400 > gz_4_6400_100.sample.sdf

python3 world_gen.py --geometry car --row 10 --col 10 --ground 1000 --sensor gazebo --lidars 10 --samples 6400 > gz_10_6400_100.sample.sdf
# 100 cars, 64000 lidars

# RTX
python3 world_gen.py --geometry car --row 30 --col 30 --ground 1000 --sensor rt --lidars 1 --samples 64000 > rt_1_64000_900.sample.sdf

python3 world_gen.py --geometry car --row 30 --col 30 --ground 1000 --sensor rt --lidars 4 --samples 64000 > rt_4_64000_900.sample.sdf

python3 world_gen.py --geometry car --row 30 --col 30 --ground 1000 --sensor rt --lidars 10 --samples 64000 > rt_10_64000_900.sample.sdf

python3 world_gen.py --geometry car --row 30 --col 30 --ground 1000 --sensor rt --lidars 20 --samples 64000 > rt_20_64000_900.sample.sdf


# gz
python3 world_gen.py --geometry car --row 30 --col 30 --ground 1000 --sensor gazebo --lidars 1 --samples 64000 > gz_1_64000_900.sample.sdf

python3 world_gen.py --geometry car --row 30 --col 30 --ground 1000 --sensor gazebo --lidars 4 --samples 64000 > gz_4_64000_900.sample.sdf

python3 world_gen.py --geometry car --row 30 --col 30 --ground 1000 --sensor gazebo --lidars 10 --samples 64000 > gz_10_64000_900.sample.sdf

#############################
# RTX
python3 world_gen.py --geometry car --row 20 --col 20 --ground 1000 --sensor rt --lidars 1 --samples 6400 > rt_1_6400_400.sample.sdf

# gz
python3 world_gen.py --geometry car --row 20 --col 20 --ground 1000 --sensor gazebo --lidars 1 --samples 6400 > gz_1_6400_400.sample.sdf


# RTX
python3 world_gen.py --geometry car --row 20 --col 30 --ground 1000 --sensor rt --lidars 1 --samples 6400 > rt_1_6400_600.sample.sdf

# gz
python3 world_gen.py --geometry car --row 20 --col 30 --ground 1000 --sensor gazebo --lidars 1 --samples 6400 > gz_1_6400_600.sample.sdf



# RTX
python3 world_gen.py --geometry car --row 20 --col 40 --ground 1000 --sensor rt --lidars 1 --samples 6400 > rt_1_6400_800.sample.sdf

# gz
python3 world_gen.py --geometry car --row 20 --col 40 --ground 1000 --sensor gazebo --lidars 1 --samples 6400 > gz_1_6400_800.sample.sdf


# RTX
python3 world_gen.py --geometry car --row 30 --col 30 --ground 1000 --sensor rt --lidars 1 --samples 6400 > rt_1_6400_900.sample.sdf

# gz
python3 world_gen.py --geometry car --row 30 --col 30 --ground 1000 --sensor gazebo --lidars 1 --samples 6400 > gz_1_6400_900.sample.sdf


# RTX
python3 world_gen.py --geometry car --row 10 --col 10 --ground 1000 --sensor rt --lidars 1 --samples 6400 > rt_1_6400_100.sample.sdf

# gz
python3 world_gen.py --geometry car --row 10 --col 10 --ground 1000 --sensor gazebo --lidars 1 --samples 6400 > gz_1_6400_100.sample.sdf

#############################
# RTX
python3 world_gen.py --geometry car --row 20 --col 20 --ground 1000 --sensor rt --lidars 1 --samples 64000 > rt_1_64000_400.sample.sdf

# gz
python3 world_gen.py --geometry car --row 20 --col 20 --ground 1000 --sensor gazebo --lidars 1 --samples 64000 > gz_1_64000_400.sample.sdf


# RTX
python3 world_gen.py --geometry car --row 20 --col 30 --ground 1000 --sensor rt --lidars 1 --samples 64000 > rt_1_64000_600.sample.sdf

# gz
python3 world_gen.py --geometry car --row 20 --col 30 --ground 1000 --sensor gazebo --lidars 1 --samples 64000 > gz_1_64000_600.sample.sdf



# RTX
python3 world_gen.py --geometry car --row 20 --col 40 --ground 1000 --sensor rt --lidars 1 --samples 64000 > rt_1_64000_800.sample.sdf

# gz
python3 world_gen.py --geometry car --row 20 --col 40 --ground 1000 --sensor gazebo --lidars 1 --samples 64000 > gz_1_64000_800.sample.sdf


# RTX
python3 world_gen.py --geometry car --row 30 --col 30 --ground 1000 --sensor rt --lidars 1 --samples 64000 > rt_1_64000_900.sample.sdf

# gz
python3 world_gen.py --geometry car --row 30 --col 30 --ground 1000 --sensor gazebo --lidars 1 --samples 64000 > gz_1_64000_900.sample.sdf


# RTX
python3 world_gen.py --geometry car --row 10 --col 10 --ground 1000 --sensor rt --lidars 1 --samples 64000 > rt_1_64000_100.sample.sdf

# gz
python3 world_gen.py --geometry car --row 10 --col 10 --ground 1000 --sensor gazebo --lidars 1 --samples 64000 > gz_1_64000_100.sample.sdf
