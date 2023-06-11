project("engine")

add_library(${PROJECT_NAME} STATIC)

file(GLOB SRC_FILES    
    "src/*.cpp"
    "src/**/*.cpp"
)

message(STATUS ${SRC_FILES})

target_sources(
    ${PROJECT_NAME}
    PRIVATE
    ${SRC_FILES}

)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
    include/

)