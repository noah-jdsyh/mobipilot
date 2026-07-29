# parse
## A. What is parse
```shell
parse is used to parse pcap file to pcd file.
```

## B. Environment
```shell
pip3 install scapy numpy pandas

# falcon_parser.py
pip3 install pybind11

# falcon_parser_cffi.py
pip3 install pybind11
```

## C. Introduction of parameter
```shell
-i input.pcap                                   # the name of input pcap file
-o output.pcd                                   # the name of output pcd file
```

## D. How to use parse
```shell
# Use falcon_parser.py to get data from pcap file to pcd file
python3 falcon_parser.py -i input.pcap -o output.pcd

# Use falcon_parser_cffi.py to get data from pcap file to pcd file
python3 falcon_parser_cffi.py -i input.pcap -o output.pcd
```

## E. Location
If you want to get detial of parse, please press [here](../apps/tools/parse/falcon_parser_cffi.py)
