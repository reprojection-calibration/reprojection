# April Tag Generation

* Generation: https://github.com/AprilRobotics/apriltag-generation
* Detection: https://github.com/AprilRobotics/apriltag

To generate the "custom" code required by apriltag-generation to generate Aprilgrid3 tags I used
[april_gen.py](april_gen.py).

## Important notes

### apriltag-generation source code change

During apriltag generation there is a check in the code that they are "complex enough". Specifically in
TagFamilyGenerator.java there is the function `boolean isComplexEnough(long v)`. To generate the v2 tag family I had to
change the complexity threshold. I hardcoded the return statement to `return energy >= 150;`.

The reason I had to do this is that the relatively large border area in relation to the data area means that the tags
have a relatively low information density which makes it harder for the detector to uniquely detect them. The calculated
threshold given the v2 tag design was about 170 which was too high, so I hardcoded it to 150 which was just low enough
to get some tags generated.

I hope this does not cause us big problem, and I write this here so that one day when someone tries to generate the
custom code using [april_gen.py](april_gen.py) they are not completely lost as to why no codes are generated. 


### Aprilgrid3 generation code

Do not forget that the code in the library only automatically generates the "data area" of the tag. The rest of the 
design like the borders and corner sharpening elements you need to do yourself! The current tag generation code is easy 
enough to understand so it's not the end of the world.

But I add this note as en explicit warning to the future warning that is confused as hell why only the data pattern is 
changing when they are trying to use a new tag design (I am speaking from experience ;).