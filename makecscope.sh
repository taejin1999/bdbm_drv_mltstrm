find . -type f -name "*.[chsS]" -print > cscope.files
cscope -b -q
ctags -L cscope.files
rm -f cscope.files
