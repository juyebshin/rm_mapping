# rm_mapping
Road Mark Mapping with baseline framework of [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)

## Install OpenCV 3.4.4
```
chmod a+x opencv-3.4.4_install.sh
./opencv-3.4.4_install.sh
```

## Build
```
chmod a+x build.sh
./build.sh
```

## Run monocular example
```
./examples/mono_apollo examples/APOLLO.yaml <path_to_sequence>
```
**<path_to_sequence>** should end with `.../ColorImage_road02/ColorImage/`
