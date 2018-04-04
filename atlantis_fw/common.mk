#******************************************************************************
#
# Compiler options
#
#******************************************************************************
CFLAGS+=-Wa,-adhlns=${COMPILER}/$(*F).lst

CCFLAGS+=-Wa,-adhlns=${COMPILER}/$(*F).lst

AFLAGS+=-Wa,-amhls=${COMPILER}/$(*F).lst

LDFLAGS+=-Wl,-Map=${COMPILER}/${TARGET_DESC}.map,--cref,--gc-sections,--no-warn-mismatch -o ${COMPILER}/${TARGET_DESC}.elf

#
# The rule to create the target directory
#
${COMPILER}:
	@mkdir ${COMPILER}

${COMPILER}/%.o: %.c
	@if [ 'x${VERBOSE}' = x ];                               \
	 then                                                    \
	     echo "  CC    ${<}";                                \
	 else                                                    \
	     echo ${CC} ${CFLAGS} -o ${@} -c ${<};               \
	 fi
	@${CC} ${CFLAGS} -o ${@} -c ${<}

${COMPILER}/%.o: %.cpp
	@if [ 'x${VERBOSE}' = x ];                               \
	 then                                                    \
	     echo "  PP    ${<}";                                \
	 else                                                    \
	     echo ${PP} ${CCFLAGS} -o ${@} -c ${<};              \
	 fi
	@${CC} ${CCFLAGS} -o ${@} -c ${<}

#******************************************************************************
#
# The rule for building the object file from each assembly source file.
#
#******************************************************************************
${COMPILER}/%.o: %.s
	@if [ 'x${VERBOSE}' = x ];                               \
	 then                                                    \
	     echo "  CC    ${<}";                                \
	 else                                                    \
	     echo ${CC} ${AFLAGS} -o ${@} -c ${<};               \
	 fi
	@${CC} ${AFLAGS} -o ${@} -c ${<}
#******************************************************************************
#
# The rule for creating an object library.
#
#******************************************************************************
${COMPILER}/%.a:
	@if [ 'x${VERBOSE}' = x ];                               \
	 then                                                    \
	     echo "  AR    ${@}";                                \
	 else                                                    \
	     echo ${AR} -cr ${@} ${^};                           \
	 fi
	@${AR} -cr ${@} ${^}

#******************************************************************************
#
# The rule for linking the application.
#
#******************************************************************************
SCATTER_${TARGET_DESC}=${LINKER_FILE}
ENTRY_${TARGET_DESC}=${TARGET_ENTRY_POINT}

${COMPILER}/%.elf:
	@${LD} -T${SCATTER_${notdir ${@:.elf=}}} --entry ${ENTRY_${notdir ${@:.elf=}}} ${LDFLAGSgcc_${notdir ${@:.elf=}}} ${LDFLAGS} -o ${@} ${^}; echo "  LD    ${@}"
	@${OBJCOPY} -O ihex ${@} ${OUTPUT_DIR}/${TARGET_DESC}.hex
	@if [ "${BUILD_TARGET}" = "S2_TARGET" ]; then \
		${PROJECT_ROOT}/ihex2bin.py ${OUTPUT_DIR}/${TARGET_DESC}.hex ${OUTPUT_DIR}/${TARGET_DESC}.bin; \
		if [ ${SBOOT_BUILD} = "ENABLE" ]; then \
			cd ${PROJECT_ROOT}/${CERT_NAME};\
			sh sb.sh ${OUTPUT_DIR}/${TARGET_DESC}.bin;\
		fi;\
	fi
	@if [ "${BUILD_TARGET}" = "S1_TARGET" ]; then \
		${OBJCOPY} -O binary ${@} ${OUTPUT_DIR}/${TARGET_DESC}.bin;\
		if [ ${SBOOT_BUILD} = "ENABLE" ]; then \
			cd ${PROJECT_ROOT}/${CERT_NAME};\
			sh sb.sh ${OUTPUT_DIR}/${TARGET_DESC}.bin;\
		fi;\
	fi
