
"""The unique identifier used for the server-client connection.

Randomly generated.
"""
UUID = "01365216-a33e-11e6-80f5-76304dec7eb7"

"""The number of bits at the start of each send indicating the length of the message."""
BYTES_FOR_PACKET_SIZE = 1

"""The maximum packet size in bytes."""
MAXIMUM_PACKET_SIZE = 256**BYTES_FOR_PACKET_SIZE - 1

"""The byte order to use when streaming ints."""
BYTE_ORDER = "big"
