from dash import Dash

# File layout inspired by https://community.plotly.com/t/splitting-callback-definitions-in-multiple-files/10583/2

# NOTE(Jack): If we do not specify the title and update behavior update here the browser tab will constantly and
# annoyingly show "Updating..." constantly.
# NOTE(Jack): We need to suppress callback exceptions because we dynamically update the layout. Just turn this to false
# if you want to understand the errors that we get.
app = Dash(title="Reprojection", update_title=None, suppress_callback_exceptions=True)

# TODO(Jack): Place meta data like this in config file or in database. For now we use globals...
IMAGE_DIMENSIONS = (512, 512)
