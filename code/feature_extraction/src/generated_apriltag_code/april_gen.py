import numpy as np

tag = np.full((6, 6), "d", dtype='U1')

tag = np.pad(tag, (1), 'constant', constant_values=("w"))
tag = np.pad(tag, (1), 'constant', constant_values=("b"))

tag = np.pad(tag, (1), 'constant', constant_values=("w"))
tag = np.pad(tag, (1), 'constant', constant_values=("w"))

tag[0:2, 0:2] = "b"
tag[0:2, -2:] = "b"
tag[-2:, 0:2] = "b"
tag[-2:, -2:] = "b"

print(tag)

tag_code = ["".join(item) for item in tag.astype(str)]
tag_code = "".join(tag_code)
print(tag_code)
