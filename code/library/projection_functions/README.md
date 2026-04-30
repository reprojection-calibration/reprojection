# Notes

## Why only one focal length (f) and not two (fx and fy)?

The article from tangram can explain it well (https://www.tangramvision.com/blog/camera-modeling-focal-length-collinearity)

I will copy and paste an excerpt from conclusion of the article here in case the above link gets taken down:

    Conclusion

    Hopefully we've demonstrated why the classical use of and in most calibration pipelines is the wrong model to pick. 
    There's always different abstractions for representing data, but using a single focal length term better matches the 
    geometry of our problem, and doesn't conflate errors in the image plane (scale differences in and directions of that 
    plane) with parameters that describe geometry outside that image plane (the focal distance). Most importantly, a 
    single focal length avoids projective compensation between the solution of that focal length and measurement errors 
    in our calibration data set.