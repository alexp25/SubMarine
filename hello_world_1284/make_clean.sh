make clean
make 2>&1 | grep -E --color=always 'error|$'
