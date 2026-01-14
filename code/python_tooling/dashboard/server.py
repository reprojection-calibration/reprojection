from dash import Dash

# File layout inspired by https://community.plotly.com/t/splitting-callback-definitions-in-multiple-files/10583/2

# NOTE(Jack): If we do not specify the title and update behavior update here the browser tab will constantly and
# annoyingly show "Updating..." constantly.
app = Dash(title='Reprojection', update_title=None)

# TODO(Jack): Do not hardcode this - giving a user the ability to interact with the file system in a gui is not so
#  trivial but can be done with some tk tools or other libraries
DB_DIR = '../../test_data/'

# TODO(Jack): Place meta data like this in config file or in database. For now we use globals...
IMAGE_DIMENSIONS = (512, 512)
