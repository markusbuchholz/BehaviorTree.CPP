cmake_minimum_required(VERSION 3.5.1)

include_directories( ../sample_nodes_dev )

set(CMAKE_DEBUG_POSTFIX "")

function(CompileExample name)
    add_executable(${name}  ${name}.cpp )
    target_link_libraries(${name} ${BEHAVIOR_TREE_LIBRARY}) #  bt_sample_nodes_dev )
endfunction()


###------------U2----------------------
CompileExample("bt_u2_ex1")
CompileExample("bt_u2_ex2")
CompileExample("bt_u2_ex3")
CompileExample("bt_u2_ex4")

###------------U3----------------------
CompileExample("bt_u3_ex1")
CompileExample("bt_u3_ex2")
CompileExample("bt_u3_ex3")
CompileExample("bt_u3_ex4")


###--------U4--------------------------
CompileExample("bt_u4_ex1")
CompileExample("bt_u4_ex2")
CompileExample("bt_u4_ex3")
CompileExample("bt_u4_ex4")
