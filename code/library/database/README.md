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

Some parts of the calibration process are inextricably linked to an image. For example extracted target features or an
optimized camera pose correspond to exactly the image that they were calculated from. We take advantage of this relation
to design the primary keys and foreign keys for image related data.

Please read external documentation on [primary keys](https://www.sqlite.org/lang_createtable.html#the_primary_key) and
[foreign keys](https://www.sqlite.org/foreignkeys.html) before reading further.

Every row in the `images` table is identified by a composite primary key `(timestmap_ns, sensor_name)`. This uniquely
identifies every single image (primary keys must be unique!). It is not possible for one camera (`sensor_name`) to take
two images at the same exact time (`timestamp_ns`).

A special feature of the `images` table is that, unlike almost all other columns in all other tables, the data columns
which holds the compressed image "blob" can be `NULL`. This allows us to remove the images if they are not needed
anymore, which keeps the database size small, while maintaining the dependent foreign key relationships.

For a more in depth understanding of the database structure please use generic analysis tools like those built into
CLion.

