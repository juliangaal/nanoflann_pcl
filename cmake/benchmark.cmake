function(add_benchmark BENCHMARK_NAME BENCHMARK_SAMPLES)
    add_test(
        NAME ${BENCHMARK_NAME}
        COMMAND ${BENCHMARK_NAME} --reporter console --reporter xml::out=${BENCHMARK_NAME}.xml --benchmark-samples ${BENCHMARK_SAMPLES}
    )
endfunction()
