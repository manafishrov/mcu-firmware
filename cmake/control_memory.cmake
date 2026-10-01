# SDK 2.3 splits the default linker script into includes. Retain its TLS,
# startup and platform sections, overriding only stack/heap placement.
function(control_replace_once variable before after)
    string(FIND "${${variable}}" "${before}" first)
    if(first EQUAL -1)
        message(FATAL_ERROR "Pico SDK linker layout changed: missing '${before}'")
    endif()
    string(REPLACE "${before}" "" without "${${variable}}")
    string(LENGTH "${${variable}}" original_length)
    string(LENGTH "${without}" remaining_length)
    string(LENGTH "${before}" match_length)
    math(EXPR removed "${original_length} - ${remaining_length}")
    if(NOT removed EQUAL match_length)
        message(FATAL_ERROR "Pico SDK linker layout changed: ambiguous '${before}'")
    endif()
    string(REPLACE "${before}" "${after}" updated "${${variable}}")
    set(${variable} "${updated}" PARENT_SCOPE)
endfunction()

function(control_reserve_stack target)
    set(sdk_includes "${PICO_SDK_PATH}/src/rp2_common/pico_standard_link/script_include")
    set(overrides "${CMAKE_CURRENT_BINARY_DIR}/control_memory")
    file(MAKE_DIRECTORY "${overrides}")
    foreach(fragment sections_stack section_heap section_end)
        set(source "${sdk_includes}/${fragment}.incl")
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${source}")
        file(READ "${source}" ${fragment})
    endforeach()
    control_replace_once(sections_stack ".stack_dummy (NOLOAD):"
        ".stack_dummy ORIGIN(RAM) + LENGTH(RAM) - 0x4000 (NOLOAD):")
    control_replace_once(sections_stack "KEEP(*(.stack*))\n    } > SCRATCH_Y"
        "KEEP(*(.stack*))\n    } > RAM")
    control_replace_once(section_heap
        "__HeapLimit = DEFINED(HEAP_LIMIT) ? HEAP_LIMIT : ORIGIN(RAM) + LENGTH(RAM);"
        "__HeapLimit = DEFINED(HEAP_LIMIT) ? HEAP_LIMIT : ORIGIN(RAM) + LENGTH(RAM) - 0x4000;")
    control_replace_once(section_end "__StackLimit = ORIGIN(RAM) + LENGTH(RAM);"
        "__StackLimit = ORIGIN(RAM) + LENGTH(RAM) - 0x4000;")
    control_replace_once(section_end "__StackTop = ORIGIN(SCRATCH_Y) + LENGTH(SCRATCH_Y);"
        "__StackTop = ORIGIN(RAM) + LENGTH(RAM);")
    string(APPEND section_end "\nASSERT(SIZEOF(.stack_dummy) == 0x4000, \"core0 stack must reserve 16 KiB\")\n")
    string(APPEND section_end "ASSERT(ADDR(.stack_dummy) == __StackBottom, \"core0 stack address mismatch\")\n")
    string(APPEND section_end "ASSERT(__HeapLimit <= __StackBottom, \"heap overlaps core0 stack\")\n")
    string(APPEND section_end "ASSERT(__end__ <= __HeapLimit, \"static data overlaps heap/stack\")\n")
    foreach(fragment sections_stack section_heap section_end)
        file(WRITE "${overrides}/${fragment}.incl" "${${fragment}}")
    endforeach()
    pico_add_linker_script_override_path(${target} "${overrides}"
        FILES sections_stack.incl section_heap.incl section_end.incl)
endfunction()
