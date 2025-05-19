# upravit a vylepsit cmake az se projekt zvetsi. ted to je ztrata casu
idf_component_register(SRCS "raketa.c" "misc/tickConversion.c" "components/minmea/minmea.c"
                    INCLUDE_DIRS "."
                )

target_compile_definitions(${COMPONENT_LIB} PRIVATE timegm=mktime)
