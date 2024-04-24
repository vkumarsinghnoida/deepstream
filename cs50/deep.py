#!/usr/bin/python3
from indoor import indoor


q = input(
    "What is the Answer  to the Great Question of Life, the Universe and Everything? ")
if q == '42' or q == 'forty-two' or q == 'forty two' or indoor(q) == 'forty-two' or indoor(q) == 'forty two':
    print('yes')
else:
    print('no')
