FILE(REMOVE_RECURSE
  "CMakeFiles/my_functionality_lib.dir/common/src/my_functional_class.o"
  "../lib/libmy_functionality_lib.pdb"
  "../lib/libmy_functionality_lib.so"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/my_functionality_lib.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
