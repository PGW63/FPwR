<div align="center">
  <h1> Find Plane with RANSAC</h1>
  <img src="https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white" alt="Ubuntu 22.04">
  <img src="https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white" alt="Python 3.10">
  <img src="https://img.shields.io/badge/ROS2-humble-22314E?logo=ros&logoColor=white" alt="ROS2 humble">
</div>

# What is this
This repository is used in RoboCup Restaurant to rotate around and determine the initial location of the bar, and to estimate the height of the plane on which an object is placed when the robot needs to approach a specific table or object for manipulation.

# Evaluation
All evaluations were conducted five times, and the bag files used for the evaluations are provided in the link below.


|object|distance|left|rigjt|front|
|---|---|---|---|---|
|dinning bar|40cm|0%|0%|100%|
|dinning bar|100cm|0%|0%|100%|
|dinning bar|130cm|0%|0%|0%|
|desk|50cm|100%|100%|100%|
|desk|90cm|0%|0%|100%|
|cabinet|50cm|0%|0%|100%|
|cabinet|80cm|0%|0%|100%|
|cabinet|120cm|0%|0%|0%|
|cabinet|150cm|0%|0%|0%|
|round table|30cm|0%|0%|100%|
|round table|60cm|0%|0%|100%|
| |||||
|Wall 1|30cm|||100%|
|Wall 2|30cm|||0%|
|person|60cm|||0%|
|person|120cm|||0%|
|30cm sofa|50cm|||0%|
|36cm bed|50cm|||0%|

## Evaluation Location
|dinning bar|desk|cabinet|round table|
|---|---|---|---|
|![dinning bar](assets/IMG_2764.jpeg)|![desk](assets/IMG_2770.jpeg)|![cabinet](assets/IMG_2769.jpeg.png)|![round table](assets/IMG_2766.jpeg.png)|

|Wall 1|Wall 2|30cm sofa|36cm bed|
|---|---|---|---|
|![Wall 1](assets/IMG_2771.jpeg)|![Wall2](assets/IMG_2765.jpeg.png)|![30cm sofa](assets/IMG_2767.jpeg)|![36cm bed](assets/IMG_2768.jpeg)|

### bag files link
[data download link](https://drive.google.com/drive/folders/1vo3dpQZectz0yMZg8Xb8tFdSDBF-ypoW?usp=drive_link)