# function to download models from own cloud
function(download_model model_path)
    if(NOT MODEL_DOWNLOAD_DIR)
        set(MODEL_DOWNLOAD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/models" PARENT_SCOPE)
    endif()

    set(model_download_path "${MODEL_DOWNLOAD_DIR}/${model_path}")

    # first check if the model already exists
    if(EXISTS ${model_download_path})
        message(STATUS "Model ${model_path} already exists")
        return()
    endif()

    # check if curl is installed
    find_program(CURL_EXECUTABLE curl)
    if(NOT CURL_EXECUTABLE)
        message(FATAL_ERROR "curl is required to download models from own cloud."
                            "Please install curl and try again.")
    endif()

    # detect if the path is nested and create the parent directories
    cmake_path(GET model_path PARENT_PATH model_parent_dir_path)
    if(model_parent_dir_path)
        file(MAKE_DIRECTORY ${MODEL_DOWNLOAD_DIR}/${model_parent_dir_path})
    endif()

    # now download the model using curl
    # FIXME(albanesg): should we use a more secure way to store this? it could be also public
    set(download_credentials "QkkJL4g4pnvSvb4:password")
    set(owncloud_webdav_url "https://polybox.ethz.ch/public.php/webdav")
    message(STATUS "Downloading model ${model_path}")
    execute_process(
        COMMAND curl -s -X GET --user ${download_credentials} -o ${model_download_path} ${owncloud_webdav_url}/${model_path}
        RESULT_VARIABLE download_result
    )

    if(NOT download_result EQUAL 0)
        message(FATAL_ERROR "Failed to download model ${model_path} from own cloud")
    endif()
endfunction()
