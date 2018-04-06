; --------------------------------------------------------------------------------------------
; Phobis bootloader. Copyright (C) 2018 by Krzysztof Szewczyk. All rights reserved!


org 0x7C00													; Entry point for every bootloader
bits 16														; We are 16bit real mode operating system

jmp short code_start										; Jump to bootloader entry point
nop															; Pad with NOP
times 3-($-$$) db 0x00										; Make sure this is the start of the BPB

; The BPB starts here

bpbOEM						db 'Phobis  '
bpbBytesPerSector			dw 512
bpbSectorsPerCluster		db 1
bpbReservedSectors			dw 68
bpbNumberOfFATs				db 2
bpbRootEntries				dw 224
bpbTotalSectors				dw 2880
bpbMedia					db 0xF8
bpbSectorsPerFAT			dw 9
bpbSectorsPerTrack			dw 18
bpbHeadsPerCylinder			dw 2
bpbHiddenSectors			dd 0
bpbTotalSectorsBig			dd 0
bsDriveNumber				db 0x00
bsUnused					db 0x00
bsExtBootSignature			db 0x29
bsSerialNumber				dd 0x12345678
bsVolumeLabel				db 'Phobis v1.0'
bsFileSystem				db 'FAT12   '

; End of BPB, begin main bootloader code

code_start:
	cli
	jmp 0x0000:initialise_cs								; Initialize CS to 0x0000 with long jmp
	initialise_cs:
		xor ax, ax
		mov ds, ax
		mov es, ax
		mov fs, ax
		mov gs, ax
		mov ax, 0x1000
		mov ss, ax
		mov sp, 0xFFF0
		sti
		mov si, loading										; Print out loading message.
		call print
		mov ax, 1											; Start from LBA sector 1
		mov bx, 0x7E00										; Load to offset 0x7E00
		mov cx, 3											; Load 3 sectors
		call read_sectors
		jc err												; Catch optional errors.
		mov si, DoneMsg
		call print											; Display done message
		jmp stage2											; Jump to stage 2

	err:
		mov si, ErrMsg
		call print

	halt:
		hlt
		jmp halt

loading 			db 0x0D, 0x0A, 'Booting Phobis...', 0x0D, 0x0A, 0x0A, 0x00
a20enablingmsg		db 'Enabling A20...', 0x00
umodemsg			db 'Enabling UMode...', 0x00
kernmsg				db 'Loading kernel...', 0x00
errormsg			db 0x0D, 0x0A, 'FATAL: System halted.', 0x00
donemsg				db 'Done loading.', 0x0D, 0x0A, 0x00

print:
	push ax													; Save registers
	push si
	mov ah, 0x0E
	.loop:
		lodsb
		test al, al											; Is is the 0x00 terminator?
		jz .done											; If it is, exit routine
		int 0x10											; Call BIOS
		jmp .loop
	.done:
		pop si												; Restore registers
		pop ax
		ret													; Exit routine

read_sector:
	push ax													; Save all GPRs
	push bx													; Prepare entering routine
	push cx
	push dx
	push dx													; Save drive number in stack
	xor dx, dx												; XOR DX for division
	div word [bpbSectorsPerTrack]							; Divide LBA / Sectors per track
	inc dl													; Adjust for sector 0
	mov byte [.absolute_sector], dl							; Save sector
	xor dx, dx												; XOR DX for division
	div word [bpbHeadsPerCylinder]							; Divide / Number of heads
	mov byte [.absolute_head], dl							; Save head
	mov byte [.absolute_track], al							; Save track
	pop dx													; Restore drive number from stack
	mov ah, 0x02											; Read sector function
	mov al, 1												; Read 1 sector
	mov ch, byte [.absolute_track]							; Use data we calculated
	mov cl, byte [.absolute_sector]
	mov dh, byte [.absolute_head]
	clc														; Clear carry for int 0x13. Buggy BIOS may not clear it on success
	int 0x13												; Call int 0x13
	.done:
		pop dx												; Restore all GPRs
		pop cx
		pop bx
		pop ax
		ret													; Exit routine
	.absolute_sector		db 0x00
	.absolute_head			db 0x00
	.absolute_track			db 0x00
read_sectors:
	push ax													; Save GPRs
	push bx
	push cx
	.loop:
		call read_sector									; Read sector
		jc .done											; If carry exit with flag
		inc ax												; Increment sector
		add bx, 512											; Add 512 to the buffer
		loop .loop											; Loop
	.done:
		pop cx												; Restore GPRs
		pop bx
		pop ax
		ret													; Exit routine


times 510-($-$$)			db 0x00							; Pad
bios_boot_sig				dw 0xAA55

stage2:
	mov si, a20enablingmsg									; Display A20 message
	call print

	call enable_a20											; Enable the A20 address line to access HMA
	jc err													; If it fails, print an error and halt

	mov si, donemsg
	call print												; Display message

	mov si, umodemsg
	call print

	enter_unreal:
		cli													; Disable interrupts
		lgdt [GDT]											; Load the GDT
		mov eax, cr0										; Enable bit 0 of cr0 and enter protected mode
		or eax, 00000001b
		mov cr0, eax
		jmp 0x08:.pmode
		.pmode:												; Now in protected mode
		mov ax, 0x10
		mov ds, ax
		mov es, ax
		mov fs, ax
		mov gs, ax
		mov ss, ax
		mov eax, cr0										; Exit protected mode
		and eax, 11111110b
		mov cr0, eax
		jmp 0x0000:.unreal_mode
		.unreal_mode:										; Now in Unreal Mode
		xor ax, ax
		mov ds, ax
		mov es, ax
		mov fs, ax
		mov gs, ax
		mov ax, 0x1000
		mov ss, ax
		mov sp, 0xFFF0
		sti													; Enable interrupts

	mov si, donemsg
	call print												; Display done message

	mov si, kernmsg											; Show loading kernel message
	call print

	mov ax, 0x2000
	mov es, ax
	mov ax, 4												; Start from LBA sector 4
	mov bx, 0x0010											; Load to offset 0x0010
	mov cx, 64												; Load 64 sectors (32 KB)
	call read_sectors

	jc err													; Catch any error

	mov si, donemsg
	call print												; Display done message

	mov ax, es
	mov ds, ax
	xor ax, ax
	not ax
	mov es, ax

	xor si, si
	xor di, di

	xor cx, cx
	not cx

	rep movsb												; Move kernel to HMA

	jmp 0xFFFF:0x0010										; Jump to the kernel.

a20_check:
	push ax													; Save registers
	push bx
	push es
	push fs
	xor ax, ax												; Set ES segment to zero
	mov es, ax
	not ax													; Set FS segment to 0xFFFF
	mov fs, ax
	mov ax, word [es:0x7DFE]								; Check using boot signature
	cmp word [fs:0x7E0E], ax								; If A20 is disabled, this should be the
															; same address as the boot signature
	je .change_values										; If they are equal, check again with another value
		.enabled:
		clc													; A20 is enabled, clear carry flag
		jmp .done
	.change_values:
		mov word [es:0x7DFE], 0xBABE						; Change the value of 0000:7DFE to 0xBABE :v)
		cmp word [fs:0x7E0E], 0xBABE						; Is FFFF:7E0E changed as well?
		jne .enabled										; If it is, A20 is enabled
		stc													; Otherwise set carry
	.done:
		mov word [es:0x7DFE], ax							; Restore boot signature
		pop fs												; Restore registers
		pop es
		pop bx
		pop ax
		ret													; Exit routine
enable_a20:
	push eax												; Save registers
	call a20_check											; Check if a20 is already enabled
	jnc .done												; If it is, we are done
	mov ax, 0x2401											; Use BIOS to try to enable a20
	int 0x15
	call a20_check											; Check again to see if BIOS succeeded
	jnc .done												; If it has, we are done
	.keyboard_method:
		cli													; Disable interrupts, they are going to destroy everything
		call .a20wait										; Use the keyboard controller to try and
		mov al, 0xAD										; open the A20 gate.
		out 0x64, al										; I saw this method on osdev wiki, used it there.
		call .a20wait
		mov al, 0xD0
		out 0x64, al
		call .a20wait2
		in al, 0x60
		push eax
		call .a20wait
		mov al, 0xD1
		out 0x64, al
		call .a20wait
		pop eax
		or al, 2
		out 0x60, al
		call .a20wait
		mov al, 0xAE
		out 0x64, al
		call .a20wait
		sti													; Enable interrupts back.
		jmp .keyboard_done
	.a20wait:
		in al, 0x64
		test al, 2
		jnz .a20wait
		ret
	.a20wait2:
		in al, 0x64
		test al, 1
		jz .a20wait2
		ret
	.keyboard_done:
		call a20_check										; Check for success
	.done:
		pop eax												; Pass flag to caller.
		ret


GDT:
	dw .GDTEnd - .GDTStart - 1								; GDT size
	dd .GDTStart											; GDT start
	.GDTStart:
	.NullDescriptor:
		dw 0x0000											; Limit
		dw 0x0000											; Base (low 16 bits)
		db 0x00												; Base (mid 8 bits)
		db 00000000b										; Access
		db 00000000b										; Granularity
		db 0x00												; Base (high 8 bits)
	.UnrealCode:
		dw 0xFFFF											; Limit
		dw 0x0000											; Base (low 16 bits)
		db 0x00												; Base (mid 8 bits)
		db 10011010b										; Access
		db 10001111b										; Granularity
		db 0x00												; Base (high 8 bits)
	.UnrealData:
		dw 0xFFFF											; Limit
		dw 0x0000											; Base (low 16 bits)
		db 0x00												; Base (mid 8 bits)
		db 10010010b										; Access
		db 10001111b										; Granularity
		db 0x00												; Base (high 8 bits)
	.GDTEnd:


times 2048-($-$$)			db 0x00							; Pad 2nd stage
