# Database

Note that this is my first attempt to design, implement and then apply a database. I am certain there are design choices
I will regret, and others I will be proud of, but most important is finding V1 and iterating until the database serves
the purpose we need it to.

My basic idea is to use a sqlite database to store the calibration record. The world is built on sql databases and they
have extremely strong support across programming languages and development environments (ex. python includes `sqlite3`
by default). What I want to avoid is saving the data into a handcrafted folder structure with `data.csv` and
`timestamp.txt` and then handwriting the ingestion code.

We need more powerful abstractions, and luckily the world provides us those tools :)

## Motivation

Existing calibration frameworks do not provide easy access to a detailed record of the calibration process. If users
want access to things like extracted target features or calculated initialization values they most likely need to hack
their way through the source code. It is my intimate experience with this pain that encourages me to make the
calibration record a first class supported process artifact.

A calibration framework should provide a record of the process. The record must be standardized and contain all the
information required to understand and debug the calibration process. The record should use standard tooling that is
available across different platforms and programming languages. The record schema must be version controlled.

## Database Record Centered Workflow

The top level calibration workflow should be viewed as the process of filling out the calibration database record. The
components of this process are:

1) Data intake plugins ingest data from source platform (ex. ROS/ROS2, px4, etc.) and writes this to the record.
    2) This design allows for a plugin like architecture. Any user can write their own data intake code to create a
       schema complaint record. This record can then be passed on to the following steps without caring about where the
       data came from.
2) Configuration options are input by the user and written to the record.
    3) At this point it is verified that the requested configuration is feasible given the data found in the record from
       step one.
3) Calibration library processes the record and performs calibration as specified by the record's configuration. During
   the calibration process intermediate data like extracted target features, poses etc. are written to the record. Parts
   of the process which are deterministic can be cached (dependent on data and config options being the same).
4) Visualization and debug tooling uses the record to provide the user feedback.

## Database Design Components

### Camera Data

A camera "frame" is identified by its timestamp and sensor name, these two pieces form the fundamental "primary
key" we use to identify camera data throughout the database. In sql we form a "composite" primary key from the
`timestamp_ns` and `sensor_name` rows. Every piece of information which is associated with a specific camera image must
reference this unique ID (directly or indirectly).

Intuitively the calibration process forms a one way chain from the images to the extracted features to the pose
initialization etc. Therefore, one would imagine that the pose table has a foreign key dependency on the extracted
features, which in turn has a foreign key dependency on the images. This would enforce that a extracted feature or pose
record can only exist if there is a corresponding entry in the image table. While this enforces the uniqueness and
record relationship record very well, it also requires that the images be present in the database. Given the large size
of images we do not want this. A 10MB record without images is nearly as valuable as a 1.01GB record with images for
many use cases.

Therefore, we must reserve the ability to remove the images from the database if they are not needed. In our linear
model this would break the relation dependency as the images cannot serve as the parent for the extracted features
anymore (Note that if we allow the image blob to be null, then maybe we can eliminate the hub and spoke design and
return to the linear relation design).

To solve this we use a "hub and spoke" model. In this data model we have a central `frames` table which has contains the
timestamp and sensor name. Both the images table (if it has data) and the extracted features have a foreign key
dependency on the frames table. The frames table acts as a central registrar of the image frames present in the
calibration process. Every quanitity that corresponds to a specific image frame must refernce (directly or indirectly)
the frames table.

Sqlite expresses dependent relationships between keys with the "foreign key" concept. 
