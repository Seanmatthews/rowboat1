if [[ $- == *i* ]]
then
    # Prompt
    NRM=`tput sgr0`
    BLD=`tput bold`
    ITL=`tput sitm`
    UL=`tput smul`
    RED=`tput setaf 1`
    GRN=`tput setaf 2`
    BLU=`tput setaf 4`
    PS1='\n\r${BLD}\u${NRM}|${UL}\h${NRM} [${BLD}${BLU}\W${NRM}] \w \n>> '
fi
