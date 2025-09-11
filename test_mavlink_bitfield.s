	.file	"test_mavlink_bitfield.c"
# GNU C17 (crosstool-NG esp-14.2.0_20241119) version 14.2.0 (xtensa-esp-elf)
#	compiled by GNU C version 8.3.0, GMP version 6.2.1, MPFR version 4.2.1, MPC version 1.2.1, isl version isl-0.26-GMP

# GGC heuristics: --param ggc-min-expand=100 --param ggc-min-heapsize=131072
# options passed: -mdynconfig=xtensa_esp32s3.so -O2
	.text
	.align	4
	.global	read_msgid
	.type	read_msgid, @function
read_msgid:
	entry	sp, 32	#
# test_mavlink_bitfield.c:21:     return msg->msgid;
	l8ui	a8, a2, 10	# *msg_3(D), *msg_3(D)
	l8ui	a9, a2, 9	# *msg_3(D), *msg_3(D)
	l8ui	a2, a2, 11	# *msg_3(D), *msg_3(D)
	slli	a8, a8, 8	# tmp48, *msg_3(D),
	or	a8, a8, a9	# tmp49, tmp48, *msg_3(D)
	slli	a2, a2, 16	# tmp51, *msg_3(D),
# test_mavlink_bitfield.c:22: }
	or	a2, a2, a8	#, tmp51, tmp49
	retw.n	
	.size	read_msgid, .-read_msgid
	.align	4
	.global	write_msgid
	.type	write_msgid, @function
write_msgid:
	entry	sp, 32	#
# test_mavlink_bitfield.c:25:     msg->msgid = id;
	extui	a9, a3, 8, 8	# tmp54, id,,
	extui	a8, a3, 16, 8	# tmp63, id,,
	s8i	a3, a2, 9	# msg_4(D)->msgid, id
	s8i	a9, a2, 10	# msg_4(D)->msgid, tmp54
	s8i	a8, a2, 11	# msg_4(D)->msgid, tmp63
# test_mavlink_bitfield.c:26: }
	retw.n	
	.size	write_msgid, .-write_msgid
	.align	4
	.global	test_vibration_id
	.type	test_vibration_id, @function
test_vibration_id:
	entry	sp, 32	#
# test_mavlink_bitfield.c:30:     msg->msgid = 241;  // MAVLINK_MSG_ID_VIBRATION
	movi.n	a8, 0	# tmp52,
	movi.n	a9, -0xf	# tmp48,
	s8i	a9, a2, 9	# msg_2(D)->msgid, tmp48
	s8i	a8, a2, 10	# msg_2(D)->msgid, tmp52
	s8i	a8, a2, 11	# msg_2(D)->msgid, tmp52
# test_mavlink_bitfield.c:31: }
	retw.n	
	.size	test_vibration_id, .-test_vibration_id
	.align	4
	.global	test_round_trip
	.type	test_round_trip, @function
test_round_trip:
	entry	sp, 32	#
# test_mavlink_bitfield.c:34: uint32_t test_round_trip(test_mavlink_message_t *msg, uint32_t id) {
	mov.n	a8, a2	# msg, tmp74
# test_mavlink_bitfield.c:35:     msg->msgid = id;
	extui	a10, a3, 8, 8	# tmp55, id,,
	extui	a9, a3, 16, 8	# tmp64, id,,
# test_mavlink_bitfield.c:37: }
	slli	a2, a3, 8	#, id,
# test_mavlink_bitfield.c:35:     msg->msgid = id;
	s8i	a3, a8, 9	# msg_4(D)->msgid, id
	s8i	a10, a8, 10	# msg_4(D)->msgid, tmp55
	s8i	a9, a8, 11	# msg_4(D)->msgid, tmp64
# test_mavlink_bitfield.c:37: }
	srli	a2, a2, 8	#,,
	retw.n	
	.size	test_round_trip, .-test_round_trip
	.section	.rodata.str1.4,"aMS",@progbits,1
	.align	4
.LC1:
	.string	"After setting msgid=241:"
	.align	4
.LC3:
	.string	"  msgid field = 0x%06X\n"
	.align	4
.LC5:
	.string	"  Raw bytes at msgid offset: "
	.align	4
.LC7:
	.string	"%02X "
	.text
	.literal_position
	.literal .LC2, .LC1
	.literal .LC4, .LC3
	.literal .LC6, .LC5
	.literal .LC8, .LC7
	.align	4
	.global	test_with_adjacent_corruption
	.type	test_with_adjacent_corruption, @function
test_with_adjacent_corruption:
	entry	sp, 32	#
# test_mavlink_bitfield.c:42:     memset(msg, 0xAA, sizeof(*msg));
	mov.n	a9, a2	# msg, msg
	movi	a10, -0x56	# tmp55,
	movi	a8, 0x114	# tmp88,
	loop	a8, .L7_LEND	# tmp88,
.L7:
	s8i	a10, a9, 0	#* msg, tmp55
	addi.n	a9, a9, 1	# msg, msg,
	.L7_LEND:		#
# test_mavlink_bitfield.c:47:     msg->sysid = 1;
	movi.n	a9, 1	# tmp58,
# test_mavlink_bitfield.c:45:     msg->magic = 0xFD;
	movi.n	a11, -3	# tmp56,
# test_mavlink_bitfield.c:49:     msg->msgid = 241;  // Should be 0x0000F1
	movi.n	a8, 0	# tmp69,
# test_mavlink_bitfield.c:45:     msg->magic = 0xFD;
	s8i	a11, a2, 2	# msg_11(D)->magic, tmp56
# test_mavlink_bitfield.c:47:     msg->sysid = 1;
	s8i	a9, a2, 7	# msg_11(D)->sysid, tmp58
# test_mavlink_bitfield.c:46:     msg->len = 32;
	movi.n	a11, 0x20	# tmp57,
# test_mavlink_bitfield.c:48:     msg->compid = 1;
	s8i	a9, a2, 8	# msg_11(D)->compid, tmp58
# test_mavlink_bitfield.c:51:     printf("After setting msgid=241:\n");
	l32r	a10, .LC2	#,
# test_mavlink_bitfield.c:49:     msg->msgid = 241;  // Should be 0x0000F1
	movi.n	a9, -0xf	# tmp65,
# test_mavlink_bitfield.c:46:     msg->len = 32;
	s8i	a11, a2, 3	# msg_11(D)->len, tmp57
# test_mavlink_bitfield.c:49:     msg->msgid = 241;  // Should be 0x0000F1
	s8i	a9, a2, 9	# msg_11(D)->msgid, tmp65
	s8i	a8, a2, 10	# msg_11(D)->msgid, tmp69
	s8i	a8, a2, 11	# msg_11(D)->msgid, tmp69
# test_mavlink_bitfield.c:51:     printf("After setting msgid=241:\n");
	call8	puts		#
# test_mavlink_bitfield.c:52:     printf("  msgid field = 0x%06X\n", msg->msgid);
	l8ui	a8, a2, 10	# *msg_11(D), *msg_11(D)
	l8ui	a9, a2, 9	# *msg_11(D), *msg_11(D)
	l8ui	a11, a2, 11	# *msg_11(D), *msg_11(D)
	slli	a8, a8, 8	# tmp78, *msg_11(D),
	or	a8, a8, a9	# tmp79, tmp78, *msg_11(D)
# test_mavlink_bitfield.c:52:     printf("  msgid field = 0x%06X\n", msg->msgid);
	l32r	a10, .LC4	#,
# test_mavlink_bitfield.c:52:     printf("  msgid field = 0x%06X\n", msg->msgid);
	slli	a11, a11, 16	# tmp81, *msg_11(D),
# test_mavlink_bitfield.c:52:     printf("  msgid field = 0x%06X\n", msg->msgid);
	or	a11, a11, a8	#, tmp81, tmp79
	call8	printf		#
# test_mavlink_bitfield.c:53:     printf("  Raw bytes at msgid offset: ");
	l32r	a10, .LC6	#,
	addi.n	a7, a2, 9	# ivtmp$26, msg,
	call8	printf		#
	l32r	a6, .LC8	#, tmp87
	addi.n	a2, a2, 13	# _33, msg,
.L8:
# test_mavlink_bitfield.c:59:         printf("%02X ", ptr[msgid_offset + i]);
	l8ui	a11, a7, 0	# MEM[(uint8_t *)_31],
	mov.n	a10, a6	#, tmp87
# test_mavlink_bitfield.c:58:     for (int i = 0; i < 4; i++) {
	addi.n	a7, a7, 1	# ivtmp$26, ivtmp$26,
# test_mavlink_bitfield.c:59:         printf("%02X ", ptr[msgid_offset + i]);
	call8	printf		#
# test_mavlink_bitfield.c:58:     for (int i = 0; i < 4; i++) {
	bne	a7, a2, .L8	# ivtmp$26, _33,
# test_mavlink_bitfield.c:61:     printf("\n");
	movi.n	a10, 0xa	#,
	call8	putchar		#
# test_mavlink_bitfield.c:62: }
	retw.n	
	.size	test_with_adjacent_corruption, .-test_with_adjacent_corruption
	.section	.rodata.str1.4
	.align	4
.LC9:
	.string	"Structure info:"
	.align	4
.LC11:
	.string	"  sizeof(msg) = %lu\n"
	.align	4
.LC13:
	.string	"  offsetof(compid) = %lu (msgid follows this)\n"
	.align	4
.LC15:
	.string	"  alignof(msg) = %lu\n"
	.align	4
.LC17:
	.string	"\nBasic tests:"
	.align	4
.LC19:
	.string	"  Written 241, read back: %u (0x%06X)\n"
	.align	4
.LC22:
	.string	"  Written 0x2F7C00, read back: 0x%06X\n"
	.align	4
.LC24:
	.string	"\nAdjacent memory test:"
	.section	.text.startup,"ax",@progbits
	.literal_position
	.literal .LC10, .LC9
	.literal .LC12, .LC11
	.literal .LC14, .LC13
	.literal .LC16, .LC15
	.literal .LC18, .LC17
	.literal .LC20, .LC19
	.literal .LC21, 3111936
	.literal .LC23, .LC22
	.literal .LC25, .LC24
	.align	4
	.global	main
	.type	main, @function
main:
	entry	sp, 320	#
# test_mavlink_bitfield.c:67:     printf("Structure info:\n");
	l32r	a10, .LC10	#,
# test_mavlink_bitfield.c:25:     msg->msgid = id;
	movi.n	a2, 0	# tmp51,
# test_mavlink_bitfield.c:67:     printf("Structure info:\n");
	call8	puts		#
# test_mavlink_bitfield.c:68:     printf("  sizeof(msg) = %lu\n", sizeof(msg));
	l32r	a10, .LC12	#,
	movi	a11, 0x114	#,
	call8	printf		#
# test_mavlink_bitfield.c:69:     printf("  offsetof(compid) = %lu (msgid follows this)\n", offsetof(test_mavlink_message_t, compid));
	l32r	a10, .LC14	#,
	movi.n	a11, 8	#,
	call8	printf		#
# test_mavlink_bitfield.c:70:     printf("  alignof(msg) = %lu\n", __alignof__(msg));
	l32r	a10, .LC16	#,
	movi.n	a11, 1	#,
	call8	printf		#
# test_mavlink_bitfield.c:73:     printf("\nBasic tests:\n");
	l32r	a10, .LC18	#,
	call8	puts		#
# test_mavlink_bitfield.c:75:     printf("  Written 241, read back: %u (0x%06X)\n", read_msgid(&msg), read_msgid(&msg));
	movi	a12, 0xf1	#,
	l32r	a10, .LC20	#,
	mov.n	a11, a12	#,
	call8	printf		#
# test_mavlink_bitfield.c:79:     printf("  Written 0x2F7C00, read back: 0x%06X\n", read_msgid(&msg));
	l32r	a11, .LC21	#,
# test_mavlink_bitfield.c:25:     msg->msgid = id;
	movi	a8, 0x7c	# tmp58,
# test_mavlink_bitfield.c:79:     printf("  Written 0x2F7C00, read back: 0x%06X\n", read_msgid(&msg));
	l32r	a10, .LC23	#,
# test_mavlink_bitfield.c:25:     msg->msgid = id;
	s8i	a8, sp, 10	# msg.msgid, tmp58
	movi.n	a8, 0x2f	# tmp65,
	s8i	a8, sp, 11	# msg.msgid, tmp65
	s8i	a2, sp, 9	# msg.msgid, tmp51
# test_mavlink_bitfield.c:79:     printf("  Written 0x2F7C00, read back: 0x%06X\n", read_msgid(&msg));
	call8	printf		#
# test_mavlink_bitfield.c:82:     printf("\nAdjacent memory test:\n");
	l32r	a10, .LC25	#,
	call8	puts		#
# test_mavlink_bitfield.c:83:     test_with_adjacent_corruption(&msg);
	mov.n	a10, sp	#,
	call8	test_with_adjacent_corruption		#
# test_mavlink_bitfield.c:86: }
	retw.n	
	.size	main, .-main
	.ident	"GCC: (crosstool-NG esp-14.2.0_20241119) 14.2.0"
