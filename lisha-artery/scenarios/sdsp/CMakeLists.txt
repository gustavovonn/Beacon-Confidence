add_artery_feature(sdsp Gateway.cc utils.cc Device.cc)

generate_opp_message(ProtocolMessages.msg TARGET sdsp DIRECTORY sdsp_msgs)

# CORREÇÃO 1: Garante que o OMNeT++ encontre os arquivos .ned do seu cenário.
add_opp_run(sdsp CONFIG omnetpp.ini NED_FOLDERS ${CMAKE_CURRENT_SOURCE_DIR})

# CORREÇÃO 2: Garante que o compilador encontre os arquivos .hpp da Vanetza.
target_include_directories(sdsp PUBLIC ${PROJECT_SOURCE_DIR}/extern/vanetza)

if(WITH_ENVMOD)
    target_sources(sdsp PUBLIC Gateway.cc utils.cc Device.cc)
    
    # CORREÇÃO 3 (A MAIS IMPORTANTE): Conecta seu programa com a biblioteca 'artery',
    # que resolve todas as outras dependências.
    target_link_libraries(sdsp PUBLIC artery envmod curl)
endif()