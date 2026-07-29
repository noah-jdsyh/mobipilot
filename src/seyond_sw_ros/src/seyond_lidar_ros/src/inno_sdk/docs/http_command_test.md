# http_command_test.py
## A. What is http_command_test.py
```shell
http_command_test.py is used to test the feasibility of commands.
```

## B. Environment
```shell
pip3 install loguru requests
```

## C. Introduction of parameter
```shell
--new                                      # test new command(1:enable 0:disable)
--old                                      # test old command(1:enable 0:disable)
```

## D. How to use http_command_test.py
```shell
# Use defalut value, test both of new command and old command
./http_command_test.py

# test new command but not old command
./http_command_test.py --new 1 --old 0
```

## E. Location
If you want to get detial of http_command_test.py, please press [here](../apps/tools/http_command_test/http_command_test.py)
