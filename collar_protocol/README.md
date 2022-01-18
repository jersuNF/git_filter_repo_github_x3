# Collar Protocol Module

Implements decode/encode of collar protocol protobuf messages. 

## Usage

To use this module make sure to set ZEPHYR_EXTRA_MODULES:
```
list(APPEND ZEPHYR_EXTRA_MODULES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/modules/collar_protocol
        )
```

There is an issue with build order when using this module. This module will generate header files that the rest of the application and modules will depend on. A custom target dependency has been created that must be added as a dependency where required, e.g.:
```
add_dependencies(app collar_protocol_headers)
```

See doxygen documentation and test implementation in tests/collar_protocol for usage of the API of the module. 