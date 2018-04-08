; --------------------------------------------------------------------------------------------
; Phobis kernel. Copyright (C) 2018 by Krzysztof Szewczyk. All rights reserved!
;
;   Licensed under the Apache License, Version 2.0 (the "License");
;   you may not use this file except in compliance with the License.
;   You may obtain a copy of the License at
;
;       http://www.apache.org/licenses/LICENSE-2.0
;
;   Unless required by applicable law or agreed to in writing, software
;   distributed under the License is distributed on an "AS IS" BASIS,
;   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
;   See the License for the specific language governing permissions and
;	limitations under the License.
; --------------------------------------------------------------------------------------------
org 0x0010							; Bootloader loads kernel here (FFFF:0010)
bits 16								; 16-bit Unreal mode

kmain:
	xor eax, eax						; Flush registers
	xor ebx, ebx
	xor ecx, ecx
	and edx, 0x000000FF					; (save boot drive)
	xor esi, esi
	xor edi, edi
	xor ebp, ebp
	cli									; Setup segments
	mov ax, KernelSpace
	mov ds, ax
	mov es, ax
	mov fs, ax
	mov gs, ax
	mov ss, ax
	mov sp, 0x7FF0
	push ds
	xor ax, ax
	mov ds, ax
	mov word [0x0200], system_call		; Hook interrupt 80h for the system API (linux 4ever)
	mov word [0x0202], KernelSpace
	mov word [0x006C], break_int		; Hook the break interrupt
	mov word [0x006E], KernelSpace
	mov word [0x0070], timer_int		; Hook the timer interrupt
	mov word [0x0072], KernelSpace
	pop ds
	sti
	mov byte [BootDrive], dl			; Save boot drive
	push 0x29							; Set current drive
	int 80h							; Prepare the screen
	push 80h							; Enter graphics mode
	int 80h
	push 0x82							; Leave graphics mode (this should fix a bug with this on some machines)
	int 80h
	reload:
	mov si, ShellName
	mov di, ShellSwitches
	push 0x14
	int 80h							; Warning: PID 1 should NEVER exit!
	mov si, ProcessWarning1
	push 0x02
	int 80h
	xor cl, cl
	xor dl, dl
	push 0x06
	int 80h							; Print exit code
	mov si, ProcessWarning2				; Print second part of message
	push 0x02
	int 80h
	push 0x18
	int 80h							; Pause
	mov dl, byte [BootDrive]			; Set the current drive to the boot drive
	push 0x29
	int 80h
	jmp reload							; Reload shell
data:
	ShellName		db	'command.com', 0x00
	ProcessWarning1	db	0x0A, "Kernel: command.com has been terminated,"
					db	0x0A, "        process exit code: ", 0x00
	ProcessWarning2	db	0x0A, "command.com will be now reloaded."
					db	0x0A, "Press a key to continue...", 0x00
	ShellSwitches	db	0x00
	BootDrive		db	0x00

; ****************************INTERNAL KERNEL PARTS BEGIN FROM HERE***************************

; --------------------------------------------------------------------------------------------
; Trasform FAT file name 'README  TXT' to a readable format 'README.TXT'.
; --------------------------------------------------------------------------------------------	
; IN:  DS:SI --> Input filename  | zero terminated
; OUT: ES:DI --> Output filename |     -||-
;
; Buffer size should always be equal to max 13 bytes,
; since max filename length is 12 (+ terminator 0x00)
fat_name_to_string:
	push ax
	push cx
	push si
	push di
	mov cx, 8						; Filename is 8 characters max
	.name_loop:
		lodsb
		cmp al, ' '					; Check for space
		je .skip_padding_loop
		stosb
		loop .name_loop
	.skip_padding_loop:
		mov al, byte [ds:si]
		test al, al
		jz .done					; No extension
		cmp al, ' '
		jne .extension
		inc si
		jmp .skip_padding_loop
	.extension:
		mov al, '.'
		stosb
	.extension_loop:
		lodsb
		cmp al, ' '
		je .done
		test al, al
		jz .done
		stosb
		jmp .extension_loop
	.done:
		xor al, al					; Add 0x00 terminator
		stosb
		pop di
		pop si
		pop cx
		pop ax
		ret
		
; --------------------------------------------------------------------------------------------
; Write entry into current directory buffer.
; --------------------------------------------------------------------------------------------
; IN: DS:SI --> filename
;     DI    --> starting cluster
;     AL    --> attributes
;     BX    --> FAT time
;     ECX   --> file size
;     DX    --> FAT date
; OUT:          Nothing
; Special: Registers preserved. Has to be wrapped over to be syscall.
fat_write_entry:
	push eax
	push ebx
	push ecx
	push edx
	push si
	push di
	push ds
	push es
	mov ax, KernelSpace
	mov es, ax
	mov word [es:.Cluster], di
	mov di, .ConvertedName			; Convert to fat name
	call string_to_fat_name
	mov ds, ax
	mov byte [.Attributes], al
	mov word [.RawTime], bx
	mov word [.RawDate], dx
	mov dword [.FileSize], ecx
	mov di, CurrentDirectoryCache
	mov word [.EntryCounter], 0x0000
	.find_empty_slot:
		inc word [.EntryCounter]
		mov ah, byte [es:di]		; Byte from the directory table, first of entry
		cmp ah, 0xE5				; Empty entry?
		jz .write_entry
		mov ah, byte [es:di]		; Byte from the directory table, first of entry
		test ah, ah					; End of table?
		jz .write_entry
		mov ax, 32					; Skip entry
		mov di, CurrentDirectoryCache
		mul word [.EntryCounter]
		add di, ax
		jmp .find_empty_slot
	.write_entry:
		mov si, .ConvertedName
		mov cx, 11					; Copy file name
		rep movsb
		mov al, byte [.Attributes]	; Store attributes
		stosb
		xor ax, ax					; Erase unused entries
		mov cx, 5
		rep stosw
		mov ax, word [.RawTime]		; Get time
		stosw
		mov ax, word [.RawDate]		; Get date
		stosw
		mov ax, word [.Cluster]		; Get cluster
		stosw
		mov eax, dword [.FileSize]	; Get size
		stosd
		pop es
		pop ds
		pop di
		pop si
		pop edx
		pop ecx
		pop ebx
		pop eax
		ret

	.EntryCounter	dw	0x0000
	.Cluster		dw	0x0000
	.FileSize		dd	0x00000000
	.RawDate		dw	0x0000
	.RawTime		dw	0x0000
	.Attributes		db	0x00
	.ConvertedName	times 12 db 0x00
	
; --------------------------------------------------------------------------------------------
; Write root directory on selected drive number.
; --------------------------------------------------------------------------------------------
; IN:  DL --> drive number
; OUT:        None
; Special: Registers preserved.
fat_write_root:
	push eax
	push ebx
	push ecx
	push edx
	push ds
	push es
	mov ax, KernelSpace
	mov ds, ax
	mov es, ax
	mov byte [.CurrentDrive], dl	; Fetch metadata from the BPB
	mov ebx, 0x0E					; Address of the Reserved sectors constant
	push 0x25
	int 80h						; Load word from address
	mov word [.StartOfFAT], ax		; Save result
	mov ebx, 0x10					; Address of the Number of FATs constant
	push 0x24
	int 80h						; Load word from address
	mov byte [.NumberOfFATs], al	; Save result
	mov ebx, 0x11					; Address of the Root entries constant
	push 0x25
	int 80h						; Load word from address
	mov word [.RootEntries], ax		; Save result
	mov ebx, 0x16					; Address of the Sectors per FAT constant
	push 0x25
	int 80h						; Load word from address
	mov word [.SizeOfFAT], ax		; Save result
	mov ax, word [.SizeOfFAT]		; Calculate the start and size of the root directory
	mov bl, byte [.NumberOfFATs]	; Start = reserved_sectors + (number_of_FATs * sectors_per_FAT)
	xor bh, bh						; Size = (root_entries * 32) / bytes_per_secto
	mul bx							; Number of fats * sector per fat in AX
	add ax, word [.StartOfFAT]		; Add reserved sectors
	mov word [.StartOfRoot], ax		; Save result in memory
	mov ax, 32						; Root entries * 32
	mul word [.RootEntries]
	xor dx, dx						; XOR DX for division
	div word [.BytesPerSector]
	mov word [.SizeOfRoot], ax		; Save result in memory
	mov bx, CurrentDirectoryCache	; Write root dir from buffer
	mov ax, word [.StartOfRoot]		; Write to here
	mov cx, word [.SizeOfRoot]		; Write this many sectors
	mov dl, byte [.CurrentDrive]	; Retrieve drive
	push 0x31
	int 80h
	pop es
	pop ds
	pop edx
	pop ecx
	pop ebx
	pop eax
	ret
	
	.SizeOfFAT				dw	0x0000
	.CurrentDrive			db	0x00
	.StartOfFAT				dw	0x0000
	.NumberOfFATs			db	0x00
	.StartOfRoot			dw	0x0000
	.SizeOfRoot				dw	0x0000
	.RootEntries			dw	0x0000
	.BytesPerSector			dw	512

; --------------------------------------------------------------------------------------------
; Delete cluster chain
; --------------------------------------------------------------------------------------------
; IN: AX --> cluster number
;	  DL --> drive number
; OUT:       None
; Special: Registers preserved.
fat12_delete_chain:
	push eax
	push ebx
	push ecx
	push edx
	push ds
	mov cx, KernelSpace					; Point DS to kernel space
	mov ds, cx
	mov word [.Cluster], ax				; Save starting cluster
	mov byte [.CurrentDrive], dl		; Save current drive
	mov ebx, 0x0E						; Address of the Reserved sectors constant
	push 0x25							; Fetch some metadata from the BPB
	int 80h							; Load word from address
	mov word [.StartOfFAT], ax			; Save result
	xor eax, eax						; Get start of FAT in bytes
	mov ax, word [.StartOfFAT]
	mov ebx, 512
	mul ebx
	mov dword [.StartOfFATInBytes], eax
	.delete_cluster:					; Delete cluster
		mov ax, word [.Cluster]			; Divide cluster by 2
		mov bx, 2
		xor dx, dx
		div bx
		add ax, word [.Cluster]			; Add this to get CLUSTER*1.5 (12 bit)
		xor ebx, ebx
		mov bx, ax
		push dx
		add ebx, dword [.StartOfFATInBytes]
		mov dl, byte [.CurrentDrive]
		push 0x25
		int 80h						; Fetch cluster
		pop dx
		cmp dx, 1						; If DX is on, we are on a split byte, and need to fetch 2 bytes,
		je .split_byte					; get the high nibble of the first, and add the second * 0x10
		push bx							; Otherwise keep the high 4 bits of AH, clear the rest
		mov bx, ax						; Save next cluster
		and bh, 00001111b
		mov word [.Cluster], bx
		pop bx
		and ax, 1111000000000000b
		push 0x32						; Write cluster to the FAT
		int 80h
		jmp .end_fetch
	.split_byte:
		push bx
		mov bx, ax
		and bl, 11110000b				; Save next cluster
		shr bx, 4
		mov word [.Cluster], bx
		pop bx
		and ax, 0000000000001111b		; Clear everything but low 4 of AL
		push 0x32						; Write cluster to the FAT
		int 80h
	.end_fetch:
		cmp word [.Cluster], 0xFF7
		jg .done
		jmp .delete_cluster
	.done:
		pop ds
		pop edx
		pop ecx
		pop ebx
		pop eax
		ret

	.CurrentDrive				db	0x00
	.Cluster					dw	0x0000
	.StartOfFAT					dw	0x0000
	.StartOfFATInBytes			dd	0x00000000

; --------------------------------------------------------------------------------------------
; Dump cluster chain to buffer
; --------------------------------------------------------------------------------------------
; IN: AX    --> cluster number
;	  ES:BX --> target buffer
;	  DL    --> drive number
; OUT:          Chain in target buffer
fat12_load_chain:
	push eax
	push ebx
	push ecx
	push edx
	push ds
	mov cx, KernelSpace					; Point DS to kernel space
	mov ds, cx
	mov word [.Cluster], ax				; Save starting cluster
	mov word [.BufferOffset], bx		; Save buffer offset
	mov byte [.CurrentDrive], dl		; Save current drive
	mov ebx, 0x0D						; Address of the Sectors per cluster constant
	push 0x24							; Fetch metadata from the BPB
	int 80h							; Load byte from address
	mov byte [.SectorsPerCluster], al
	mov ebx, 0x0E						; Address of the Reserved sectors constant
	push 0x25
	int 80h							; Load word from address
	mov word [.StartOfFAT], ax			; Save result
	mov ebx, 0x10						; Address of the Number of FATs constant
	push 0x24
	int 80h							; Load word from address
	mov byte [.NumberOfFATs], al		; Save result
	mov ebx, 0x11						; Address of the Root entries constant
	push 0x25
	int 80h							; Load word from address
	mov word [.RootEntries], ax			; Save result
	mov ebx, 0x16						; Address of the Sectors per FAT constant
	push 0x25
	int 80h							; Load word from address
	mov word [.SizeOfFAT], ax			; Save result
	mov ax, word [.SectorsPerCluster]	; Get sectors per cluster in bytes
	mov bx, 512
	mul bx
	mov word [.SectorsPerClusterInBytes], ax
	xor eax, eax						; Get start of FAT in bytes
	mov ax, word [.StartOfFAT]
	mov ebx, 512
	mul ebx
	mov dword [.StartOfFATInBytes], eax	; Calculate the start and size of the root directory
	mov ax, word [.SizeOfFAT]			; Start = reserved_sectors + (number_of_FATs * sectors_per_FAT)
	mov bl, byte [.NumberOfFATs]		; Size = (root_entries * 32) / bytes_per_sector
	xor bh, bh							; Number of fats * sector per fat in AX
	mul bx
	add ax, word [.StartOfFAT]			; Add reserved sectors
	mov word [.StartOfRoot], ax			; Save result in memory
	mov ax, 32; Root entries * 32
	mul word [.RootEntries]
	xor dx, dx							; XOR DX for division
	div word [.BytesPerSector]
	mov word [.SizeOfRoot], ax			; Save result in memory
	mov ax, word [.StartOfRoot]			; Start of data = (Start of root - 2) + size of root
	sub ax, 2							; Subtract 2 to get LBA
	add ax, word [.SizeOfRoot]
	mov word [.DataStart], ax			; Load chain
	mov ax, word [.Cluster]				; Prepare to enter loop
	mov bx, word [.BufferOffset]
	.cluster_loop:
		mov dl, byte [.CurrentDrive]		; Retrieve current drive
		cmp ax, 0xFF7						; Is the last cluster?
		jg .done							; If yes, we finished
		mul byte [.SectorsPerCluster]		; Multiply ax by the sectors per cluster
		add ax, word [.DataStart]			; Add the data start offset
		xor cx, cx
		mov cl, byte [.SectorsPerCluster]
		push 0x23							; Read
		int 80h
		add bx, word [.SectorsPerClusterInBytes]	; Move buffer up the bytes per cluster size
		push bx
		mov ax, word [.Cluster]				; Divide cluster by 2
		mov bx, 2
		xor dx, dx
		div bx
		add ax, word [.Cluster]				; Add this to get CLUSTER*1.5 (12 bit)
		xor ebx, ebx
		mov bx, ax
		push dx
		add ebx, dword [.StartOfFATInBytes]
		mov dl, byte [.CurrentDrive]
		push 0x25
		int 80h							; Fetch cluster
		pop dx
		cmp dx, 1							; If DX is on, we are on a split byte, and need to fetch 2 bytes,
		je .split_byte						; get the high nibble of the first, and add the second * 0x10
		and ah, 00001111b					; Otherwise clear the high 4 bits of AH
		jmp .end_fetch
	.split_byte:
		and al, 11110000b					; Clear low 4 of AL
		shr ax, 4							; Shift right a nibble
	.end_fetch:
		pop bx
		mov word [.Cluster], ax				; Save current cluster
		jmp .cluster_loop
	.done:
		pop ds
		pop edx
		pop ecx
		pop ebx
		pop eax
		ret

	.DataStart					dw	0x0000
	.SizeOfFAT					dw	0x0000
	.CurrentDrive				db	0x00
	.Cluster					dw	0x0000
	.BufferOffset				dw	0x0000
	.SectorsPerCluster			db	0x00
	.SectorsPerClusterInBytes	dw	0x0000
	.StartOfFAT					dw	0x0000
	.StartOfFATInBytes			dd	0x00000000
	.NumberOfFATs				db	0x00
	.StartOfRoot				dw	0x0000
	.SizeOfRoot					dw	0x0000
	.RootEntries				dw	0x0000
	.BytesPerSector				dw	512

; --------------------------------------------------------------------------------------------
; Read a floppy sector with specified LBA address. Warning: internal routine, not suited for
; direct use in external API.
; --------------------------------------------------------------------------------------------
; IN:  AX --> LBA sector to load
; 	   DL --> Drive number
; 	   ES --> Buffer segment
; 	   BX --> Buffer offset
; OUT:		  Carry on error
floppy_read_sector:
	push ax									; Save all GPRs
	push bx									; Prepare entering routine
	push cx
	push dx
	push bx									; Save target buffer in stack
	push dx									; Save drive number in stack
	xor dx, dx								; XOR DX for division
	mov bx, 18								; Divide LBA / Sectors per track (18 on 1.44 floppy)
	div bx									; LBA to CHS
	inc dl									; Adjust for sector 0
	mov byte [.absolute_sector], dl			; Save sector
	xor dx, dx								; XOR DX for division
	mov bx, 2
	div bx									; Divide / Number of heads (2)
	mov byte [.absolute_head], dl			; Save head
	mov byte [.absolute_track], al			; Save track
	pop dx									; Restore drive number from stack
	pop bx									; Restore target buffer from stack
	mov ah, 0x02							; Read sector function
	mov al, 1								; Read 1 sector
	mov ch, byte [.absolute_track]			; Use data we calculated
	mov cl, byte [.absolute_sector]			; Prepare registers for BIOS int 0x13
	mov dh, byte [.absolute_head]
	clc										; Clear carry for int 0x13 because some BIOSes may not clear it on success
	int 0x13								; Call int 0x13
	.done:
		pop dx								; Restore all GPRs
		pop cx
		pop bx
		pop ax
		ret									; Exit routine

	.absolute_sector		db 0x00
	.absolute_head			db 0x00
	.absolute_track			db 0x00

; --------------------------------------------------------------------------------------------
; Write floppy sector with specified LBA adress. Warning: used as internal routine.
; Shares repetitive code with floppy_read_sector method.
; --------------------------------------------------------------------------------------------
; IN:  AX --> LBA sector to write
;      DL --> Drive number
;      ES --> Buffer segment
;      BX --> Buffer offset
; OUT: 	   	  Carry on error
floppy_write_sector:
	push ax									; Save all GPRs
	push bx									; Prepare entering routine
	push cx
	push dx
	push bx									; Save target buffer in stack
	push dx									; Save drive number in stack
	mov byte [CacheStatus], 0x00			; Invalidate cache
	xor dx, dx								; XOR DX for division
	mov bx, 18								; Divide LBA / Sectors per track (18 on 1.44 floppy)
	div bx									; LBA to CHS
	inc dl									; Adjust for sector 0
	mov byte [.absolute_sector], dl			; Save sector
	xor dx, dx								; XOR DX for division
	mov bx, 2
	div bx									; Divide / Number of heads (2)
	mov byte [.absolute_head], dl			; Save head
	mov byte [.absolute_track], al			; Save track
	pop dx									; Restore drive number from stack
	pop bx									; Restore target buffer from stack
	mov ah, 0x03							; Write sector function
	mov al, 1								; Write 1 sector
	mov ch, byte [.absolute_track]			; Use data we calculated
	mov cl, byte [.absolute_sector]			; Prepare registers for BIOS int 0x13
	mov dh, byte [.absolute_head]
	clc										; Clear carry for int 0x13 because some BIOSes may not clear it on success
	int 0x13								; Call int 0x13
	.done:
		pop dx								; Restore all GPRs
		pop cx
		pop bx
		pop ax
		ret									; Exit routine
	.absolute_sector		db 0x00
	.absolute_head			db 0x00
	.absolute_track			db 0x00
	
; --------------------------------------------------------------------------------------------
; Convert relative path to absolute one.
; --------------------------------------------------------------------------------------------
; IN:  DS:SI --> Relative path
; OUT: ES:DI --> Absolute path
; Special: Preserves registers.
path_converter:
	push ax										; Save used registers
	push bx
	push cx
	push dx
	push si
	push di
	push gs
	mov ax, KernelSpace							; GS to kernel
	mov gs, ax
	mov word [gs:.TargetBuffer], di				; Save address of target location
	lodsb										; Check if the path is absolute
	cmp al, '/'
	je .main
	dec si
	push 0x2E									; Dump the pwd in the target buffer
	int 80h
	push 0x2D									; Get to end of string
	int 80h
	.main:
		mov bx, .DirectoryName					; Put a temporary buffer in BX to check for . or .. entries
	.name_loop:
		lodsb
		cmp al, '/'
		je .directory
		test al, al
		jz .directory
		mov byte [gs:bx], al
		inc bx
		jmp .name_loop
	.directory:
		mov byte [gs:bx], 0x00					; Add terminator to buffer
		push si									; Save these registers
		push di
		push ds
		push es
		mov ax, KernelSpace						; Point es and ds to kernel
		mov ds, ax
		mov es, ax
		mov si, .DirectoryName					; Check for reserved entries (. and ..)
		mov di, .DotEntry
		push 0x08								; Compare strings
		int 80h
		cmp dl, 0x01
		je .dot_entry
		mov di, .DotDotEntry
		push 0x08
		int 80h
		cmp dl, 0x01
		je .dot_dot_entry						; If not a reserved entry
		pop es
		pop ds
		pop di
		pop si
		mov bx, .DirectoryName
		mov ax, di								; check if root dir
		dec ax
		cmp ax, word [gs:.TargetBuffer]
		je .no_slash
		mov al, '/'								; Add slash (meaning root directory)
		stosb
	.no_slash:
		.copy_to_buffer_loop:
			mov al, byte [gs:bx]
			test al, al
			jz .not_reserved_done
			inc bx
			stosb
			jmp .copy_to_buffer_loop
		.not_reserved_done:
			mov byte [es:di], 0x00				; Put a terminator, just in case
			mov al, byte [ds:(si-1)]
			test al, al							; If end of string, done
			jz .done
			jmp .main
		.dot_entry:								; If a . entry
			pop es
			pop ds
			pop di
			pop si
			mov al, byte [ds:(si-1)]
			test al, al							; If end of string, done
			jz .done
			jmp .main
		.dot_dot_entry:							; If a .. entry
			pop es
			pop ds
			pop di
			pop si
			.remove_last_dir_loop:
				mov al, byte [es:di]			; check if root dir
				mov byte [es:di], 0x00
				cmp al, '/'
				je .remove_dir_done
				mov ax, di
				dec ax
				cmp ax, word [gs:.TargetBuffer]
				je .remove_dir_done
				dec di
				jmp .remove_last_dir_loop
			.remove_dir_done:
				mov al, byte [ds:(si-1)]
				test al, al						; If end of string, done
				jz .done
				jmp .main
		.done:
			pop gs
			pop di
			pop si
			pop dx
			pop cx
			pop bx
			pop ax
			push 0x15							; Convert to uppercase
			int 80h
			ret


	.DirectoryName		times 13 db 0x00
	.DotEntry			db '.', 0x00
	.DotDotEntry		db '..', 0x00
	.TargetBuffer		dw 0x0000

; --------------------------------------------------------------------------------------------
; Transform readable path string to FAT entry filename. 'file.txt' --> 'FILE    TXT'
; --------------------------------------------------------------------------------------------
; IN:  DS:SI --> Input filename
; OUT: ES:DI --> Output filename
; Special: 		 Registers preserved.
;				 Buffer size has to be >= 12 (11 characters + null terminator)
string_to_fat_name:
	push ax
	push cx
	push dx
	push si
	push di
	push di
	push es
	mov ax, KernelSpace
	mov es, ax
	mov di, .dot_dot_entry
	push 0x08
	int 80h
	cmp dl, 0x01
	je .ignore
	mov di, .dot_entry
	push 0x08
	int 80h
	cmp dl, 0x01
	je .ignore
	pop es
	pop di
	mov cl, 11
	.clear_buffer:
		cmp cl, 0
		je .main
		mov al, ' '
		stosb
		dec cl
		jmp .clear_buffer
	.main:
		pop di
		push di
		mov cl, 11
	.loop:
		lodsb										; Byte from SI
		cmp al, '.'									; Is '.'?
		je .pad_and_extension
		cmp al, 0x00								; Is terminator?
		je .pad_and_convert
		cmp cl, 3									; Too may characters?
		je .convert									; Quit
		stosb										; Save in DI
		dec cl
		jmp .loop
	.pad_and_extension:
		sub cl, 3									; Sub extension size from padding
	.loop1:
		cmp cl, 0
		je .put_extension
		mov al, ' '									; Pad with spaces
		dec cl
		stosb
		jmp .loop1
	.put_extension:
		mov cl, 3
	.loop2:
		cmp cl, 0
		je .convert
		lodsb										; Byte from SI
		cmp al, 0x00								; Is 0x00?
		je .pad_and_convert
		stosb										; Store in DI
		dec cl
		jmp .loop2
	.pad_and_convert:
		cmp cl, 0
		je .convert
		mov al, ' '									; Pad with spaces
		stosb
		dec cl
		jmp .pad_and_convert
	.convert:
		pop di										; Reset DI
		push di
		add di, 11
		mov byte [es:di], 0x00						; Add 0x00 terminator
		sub di, 11
		push 0x15
		int 80h									; Lower to uppercase
	.done:
		pop di
		pop si
		pop dx
		pop cx
		pop ax
		ret
	.ignore:
		pop es
		pop di
	mov cx, 11
	.clear_buffer1:
		mov al, ' '
		stosb
		loop .clear_buffer1
		pop di
		push di
		.copy_string:
			lodsb
			test al, al
			jz .done
			stosb
			jmp .copy_string
	.dot_dot_entry		db	'..', 0x00
	.dot_entry			db	'.', 0x00

; --------------------------------------------------------------------------------------------
; Check does file end with .BIN
; --------------------------------------------------------------------------------------------
; Special: Registers preserved.
check_bin_extension:
	push ax
	push dx
	push esi
	push edi
	push es
	mov ax, ds      								; Convert file name to uppercase
	mov es, ax
	mov edi, esi
	push 0x15
	int 80h
	mov ax, KernelSpace
	mov es, ax
	.loop:
		a32 o32 lodsb
		cmp al, '.'
		je .check_extension
		test al, al
		jz .no_bin
		jmp .loop
		.check_extension:
			mov edi, .bin_string
			push 0x08
			int 80h
			test dl, dl
			jz .no_bin
		.bin:
			clc
			jmp .done
		.no_bin:
			stc
	.done:
		pop es
		pop edi
		pop esi
		pop dx
		pop ax
		ret
	.bin_string db 'COM', 0x00
	
; --------------------------------------------------------------------------------------------
; Clear cursor. Named such because of consistency, actual name might be misleading.
; --------------------------------------------------------------------------------------------
; IN:      None
; OUT:     None
; Special: Registers preserved.
clear_cursor:
	push ax
	push di
	push es
	push ds
	mov ax, 0xB800
	mov es, ax
	mov ax, KernelSpace
	mov ds, ax
	mov di, word [CursorLocation]
	mov al, byte [es:di]
	mov ah, byte [CharAttributes]
	stosw
	pop ds
	pop es
	pop di
	pop ax
	ret

; --------------------------------------------------------------------------------------------
; Draw cursor. Named such because of consistency, actual name might be misleading.
; --------------------------------------------------------------------------------------------
; IN:      None
; OUT:     None
; Special: Registers preserved.
draw_cursor:
	push ax
	push di
	push es
	push ds
	mov ax, 0xB800
	mov es, ax
	mov ax, KernelSpace
	mov ds, ax
	mov di, word [CursorLocation]
	mov al, byte [es:di]
	mov ah, byte [CursorAttributes]
	stosw
	pop ds
	pop es
	pop di
	pop ax
	ret

; --------------------------------------------------------------------------------------------
; Clear directory cache.
; --------------------------------------------------------------------------------------------
; IN:      None
; OUT:     None
; Special: Registers preserved.
erase_dir_cache:
	push ax
	push es
	push di
	mov ax, KernelSpace
	mov es, ax
	mov di, CurrentDirectoryCache
	mov cx, 0x2000
	xor ax, ax
	rep stosw
	pop di
	pop es
	pop ax
	ret

; --------------------------------------------------------------------------------------------
; Delete FAT chain. Should run procedure based on FS (WIP). Look fat12_delete_chain.
; --------------------------------------------------------------------------------------------
; IN:      Inherited
; OUT:     Inherited
; Special: Inherited
fat_delete_chain:
	call fat12_delete_chain
	ret
	
; --------------------------------------------------------------------------------------------
; Return FAT metadata of entry in current working directory.
; --------------------------------------------------------------------------------------------
; IN:  DS:SI--> file/dir name
;      DL   --> 1 for directory, 0 for file
; OUT: SI   --> starting cluster, 0x0000 if not found
;      AL   --> 0x00 if success, 0xFF if not found
;      BX   --> raw FAT time
;      ECX  --> file size
;      DX   --> raw FAT date
; Special: Registers preserved.
fat_get_metadata:
	push di
	push ds
	push es
	push eax
	mov ax, KernelSpace
	mov es, ax
	mov di, .ConvertedName								; Convert to fat name
	call string_to_fat_name
	mov ds, ax
	mov byte [.DirectoryFlag], dl
	mov byte [.Success], 0x00
	mov di, CurrentDirectoryCache
	mov word [.EntryCounter], 0x0000
	.next_entry:
		mov si, .ConvertedName
		inc word [.EntryCounter]
		mov ah, byte [es:di]								; Byte from the directory table, first of entry
		test ah, ah											; End of table?
		jz .not_found
		mov cx, 11
	.check_name_loop:
		lodsb												; Byte from the file name
		mov ah, byte [es:di]								; Byte from table
		inc di
		cmp al, ah
		jne .skip_entry
		loop .check_name_loop 								; File found
		mov al, byte [di]
		and al, 0x10										; Directory?
		jnz .directory
		cmp byte [.DirectoryFlag], 0x00
		jne .skip_entry
		jmp .get_metadata
	.directory:
		cmp byte [.DirectoryFlag], 0x01
		jne .skip_entry
	.get_metadata:
		add di, 11											; Get raw creation time
		mov ax, word [es:di]
		mov word [.RawTime], ax
		add di, 2											; Get raw creation date
		mov ax, word [es:di]
		mov word [.RawDate], ax
		add di, 2											; Retrieve starting cluster
		mov ax, word [es:di]
		mov word [.Cluster], ax
		add di, 2											; Get file size
		mov eax, dword [es:di]
		mov dword [.FileSize], eax
		jmp .done
	.skip_entry:
		mov ax, 32
		mov di, CurrentDirectoryCache
		mul word [.EntryCounter]
		add di, ax
		jmp .next_entry
	.not_found:
		mov byte [.Success], 0xFF
	.done:
		pop eax
		mov al, byte [.Success]
		mov si, word [.Cluster]
		mov bx, word [.RawTime]
		mov ecx, dword [.FileSize]
		mov dx, word [.RawDate]
		pop es
		pop ds
		pop di
		ret

	.Success		db	0x00
	.EntryCounter	dw	0x0000
	.Cluster		dw	0x0000
	.FileSize		dd	0x00000000
	.RawDate		dw	0x0000
	.RawTime		dw	0x0000
	.DirectoryFlag	db	0x00
	.ConvertedName	times 12 db 0x00

; --------------------------------------------------------------------------------------------
; Load FAT chain. Should run procedure based on FS (WIP). Look fat12_load_chain.
; --------------------------------------------------------------------------------------------
; IN:      Inherited
; OUT:     Inherited
; Special: Inherited
fat_load_chain:
	call fat12_load_chain
	ret

; --------------------------------------------------------------------------------------------
; Load root directory into directory buffer.
; --------------------------------------------------------------------------------------------
; IN:      DL --> drive number
; OUT:     None
; Special: Registers preserved.
fat_load_root:
	push eax
	push ebx
	push ecx
	push edx
	push ds
	push es
	mov ax, KernelSpace
	mov ds, ax
	mov es, ax
	mov byte [.CurrentDrive], dl				; Fetch some metadata from the BPB
	mov ebx, 0x0E								; Address of the Reserved sectors constant
	push 0x25
	int 80h									; Load word from address
	mov word [.StartOfFAT], ax					; Save result
	mov ebx, 0x10								; Address of the Number of FATs constant
	push 0x24
	int 80h									; Load word from address
	mov byte [.NumberOfFATs], al				; Save result
	mov ebx, 0x11								; Address of the Root entries constant
	push 0x25
	int 80h									; Load word from address
	mov word [.RootEntries], ax					; Save result
	mov ebx, 0x16								; Address of the Sectors per FAT constant
	push 0x25
	int 80h									; Load word from address
	mov word [.SizeOfFAT], ax					; Save result
	mov ax, word [.SizeOfFAT]					; Calculate the start and size of the root directory
	mov bl, byte [.NumberOfFATs]				; Start = reserved_sectors + (number_of_FATs * sectors_per_FAT)
	xor bh, bh									; Size = (root_entries * 32) / bytes_per_sector
	mul bx										; Number of fats * sector per fat in AX
	add ax, word [.StartOfFAT]					; Add reserved sectors
	mov word [.StartOfRoot], ax					; Save result in memory
	mov ax, 32									; Root entries * 32
	mul word [.RootEntries]
	xor dx, dx									; XOR DX for division
	div word [.BytesPerSector]
	mov word [.SizeOfRoot], ax					; Save result in memory
	mov bx, CurrentDirectoryCache				; Load root dir into buffer
	mov ax, word [.StartOfRoot]					; Load from here
	mov cx, word [.SizeOfRoot]					; Load this many sectors
	mov dl, byte [.CurrentDrive]				; Retrieve drive
	push 0x23
	int 80h
	pop es
	pop ds
	pop edx
	pop ecx
	pop ebx
	pop eax
	ret

	.SizeOfFAT				dw	0x0000
	.CurrentDrive			db	0x00
	.StartOfFAT				dw	0x0000
	.NumberOfFATs			db	0x00
	.StartOfRoot			dw	0x0000
	.SizeOfRoot				dw	0x0000
	.RootEntries			dw	0x0000
	.BytesPerSector			dw	512

; ****************************EXTERNAL KERNEL PARTS BEGIN FROM HERE***************************

; --------------------------------------------------------------------------------------------
; Clear frame buffer
; --------------------------------------------------------------------------------------------
; IN:  None
; OUT: None
; Special: Registers preserved
clear_frame_buffer:
	push ax
	push ecx
	push edi
	push es
	xor ax, ax
	mov es, ax
	mov edi, FrameBuffer
	xor al, al
	xor ecx, ecx
	not cx
	a32 o32 rep stosb
	pop es
	pop edi
	pop ecx
	pop ax
	iret

; --------------------------------------------------------------------------------------------
; Compare two strings.
; --------------------------------------------------------------------------------------------
; IN:  DS:ESI --> String 1
;      ES:EDI --> String 2
; OUT: DL     --> 1 if equal, 0 if not
; Special: Registers preserved
compare_strings:
	push ax
	push esi
	push edi
	.loop:
		a32 o32 lodsb
		mov ah, byte [es:edi]
		inc edi
		cmp al, ah
		jne .not_equal
		test al, al
		jz .equal
		jmp .loop
	.not_equal:
		xor dl, dl
		jmp .done
	.equal:
		mov dl, 0x01
	.done:
		pop edi
		pop esi
		pop ax
		iret

; --------------------------------------------------------------------------------------------
; Cut string in desired place, adding null terminator.
; --------------------------------------------------------------------------------------------
; IN:  DS:ESI --> String
;      BL     --> Character to cut at.
; OUT: DS:ESI --> string cut
;	   DS:EBX --> starting position of the second part of the string
; Special: Registers preserved
cut_string:
	push ax
	push esi
	.loop:
		a32 o32 lodsb
		cmp al, bl
		je .cut_string
		test al, al
		jz .fail
		jmp .loop
	.cut_string:
		dec esi
		xor al, al
		mov byte [ds:esi], al
		inc esi
		jmp .done
	.fail:
		dec esi
	.done:
		mov ebx, esi
		pop esi
		pop ax
		iret

; --------------------------------------------------------------------------------------------
; Scan content of current directory.
; --------------------------------------------------------------------------------------------
; IN:  AX    --> Directory entry number
; OUT: AX    --> Raw time
;      BX    --> Raw date
;      ES:DI --> File name
;      ECX   --> Size
;      DH    --> Directory flag (0x00 if file, 0xFF if directory)
;      DL    --> 0xFF if entry not found, 0x00 if found
; Special:       Registers preserved.
directory_scanner:
	push si
	push di
	push ds
	push es
	mov bx, KernelSpace
	mov ds, bx
	mov word [.EntryNumber], ax
	mov word [.TargetBuffer], di
	mov ax, es
	mov word [.TargetSegment], ax
	mov word [.EntryCounter], 0x0000
	mov es, bx
	mov si, CurrentDirectoryCache
	.next_entry:
		mov di, .FatNameBuffer
		lodsb												; Byte from the directory table, first of entry
		dec si
		test al, al											; End of table?
		jz .failure
		mov cx, 11
		rep movsb
		xor al, al
		stosb										 		; Check for special reserved entries
		mov al, byte [si-11]								; Check for deleted entry
		cmp al, 0xE5
		je .skip_entry
		mov al, byte [si]									; Check for a vfat entry (ignore it)
		cmp al, 0x0F
		je .skip_entry
		mov al, byte [si]									; Check for a dir entry
		and al, 0x10
		jz .no_directory
		mov byte [.DirectoryFlag], 0xFF
		jmp .directory_check_done
	.no_directory:
		mov byte [.DirectoryFlag], 0x00
	.directory_check_done:
		mov ax, word [si+11]								; Get time
		mov word [.RawTime], ax
		mov ax, word [si+13]								; Get date
		mov word [.RawDate], ax
		mov eax, dword [si+17]								; Get size
		mov dword [.FileSize], eax
		mov ax, word [.EntryNumber]
		cmp word [.EntryCounter], ax
		je .success
		inc word [.EntryCounter]
	.skip_entry:
		add si, 21											; Get to the next entry
		jmp .next_entry
	.failure:
		xor dl, dl
		not dl
		jmp .done
	.success:
		mov si, .FatNameBuffer								; Convert name
		mov di, word [.TargetBuffer]
		mov ax, word [.TargetSegment]
		mov es, ax
		call fat_name_to_string
		xor dl, dl
	.done:
		mov dh, byte [.DirectoryFlag]
		mov ax, word [.RawTime]
		mov bx, word [.RawDate]
		mov ecx, dword [.FileSize]
		pop es
		pop ds
		pop di
		pop si
		iret

	.DirectoryFlag		db	0x00
	.EntryNumber		dw	0x0000
	.EntryCounter		dw	0x0000
	.TargetBuffer		dw	0x0000
	.TargetSegment		dw	0x0000
	.RawDate			dw	0x0000
	.RawTime			dw	0x0000
	.FileSize			dd	0x00000000
	.FatNameBuffer		times 12 db 0x00

; --------------------------------------------------------------------------------------------
; Disable text mode cursor.
; --------------------------------------------------------------------------------------------
; IN:  		None
; OUT: 		None
; Special:  Registers preserved.
disable_cursor:
	push ax
	push ds
	mov ax, KernelSpace
	mov ds, ax
	call clear_cursor
	mov byte [CursorStatus], 0x00
	pop ds
	pop ax
	iret
	
; --------------------------------------------------------------------------------------------
; Draw a line in graphic mode. Using https://en.wikipedia.org/wiki/Bresenham's_line_algorithm
; --------------------------------------------------------------------------------------------
; IN: DL --> Color
;     BX --> X1
;     CL --> Y1
;     AX --> X2
;     CH --> Y2
; OUT: 		 None
; Special:   Registers preserved.
draw_line:
	push ax
	push bx
	push cx
	push dx
	push ds
	push ax
	mov ax, KernelSpace
	mov ds, ax
	pop ax
	mov byte [.Colour], dl
	mov word [.dx], ax
	sub word [.dx], bx
	push cx
	xor cl, cl
	shr cx, 8
	mov word [.dy], cx
	pop cx
	push cx
	xor ch, ch
	sub word [.dy], cx
	pop cx
	push ax
	push bx
	mov ax, word [.dy]
	mov bx, 2
	mul bx
	sub ax, word [.dx]
	mov word [.D], ax
	pop bx
	pop ax
	.loop:
		cmp bx, ax
		je .done
		inc bx
		mov dl, byte [.Colour]
		push 0x81
		int 80h
		cmp word [.D], 0x0000
		jg .do_stuff
	.stuff_done:
		mov dx, word [.dy]
		add word [.D], dx
		jmp .loop
	.do_stuff:
		inc cl
		mov dx, word [.D]
		sub dx, word [.dx]
		mov word [.D], dx
		jmp .stuff_done
	.done:
		pop ds
		pop dx
		pop cx
		pop bx
		pop ax
		iret


	.Colour	db	0x00
	.D		dw	0x0000
	.dx		dw	0x0000
	.dy		dw	0x0000

; --------------------------------------------------------------------------------------------
; Draw pixel in graphic mode. Warning: HIGH OVERHEAD. Propably slow.
; --------------------------------------------------------------------------------------------
; IN: DL --> Color
;	  BX --> X
;	  CL --> Y
; OUT: 		 None
; Special: Registers preserved.
draw_pixel:
	push eax
	push ebx
	push ecx
	push edx
	push es
	push edi
	push dx
	xor eax, eax			; Point ES to a flat segment
	mov es, ax
	mov	edi, FrameBuffer	; Point EDI to frame buffer
	mov ax, 320    			; Multiply Y by 320
	xor	ch, ch
	mul	cx
	add	edi, eax			; Add to frame buffer location
	and	ebx, 0x0000FFFF
	add edi, ebx       		; Add the X coordinate
	pop dx
	mov byte [es:edi], dl  	; Set the pixel to the color specified
	pop edi
	pop es
	pop edx
	pop ecx
	pop ebx
	pop eax
	iret

; --------------------------------------------------------------------------------------------
; Draw sprite graphic mode. Warning: Sprite has to have valid format.
; --------------------------------------------------------------------------------------------
; IN:  DS:SI --> Location of the sprite data buffer.
;      BX    --> X
;      CL    --> Y
; OUT: None
; Sprite data formatting rules:
;	   SpriteXLength		word
;	   SpriteYLength		byte
;	   SpriteData:         byte[] - pixel colours
draw_sprite:
	push ax
	push bx
	push cx
	push dx
	push si
	push es
	mov ax, KernelSpace
	mov es, ax
	mov word [es:.OriginalBX], bx
	lodsw
	mov word [es:.SpriteX], ax
	add word [es:.SpriteX], bx
	lodsb
	mov byte [es:.SpriteY], al
	add byte [es:.SpriteY], cl
	.loop:
		lodsb
		cmp al, 0xFF
		je .compression
		mov dl, al
		cmp bx, word [es:.SpriteX]
		je .next_row
		push 0x81
		int 80h
		inc bx
		jmp .loop
		.next_row:
			mov bx, word [es:.OriginalBX]
			inc cl
			cmp cl, byte [es:.SpriteY]
			je .done
			push 0x81
			int 80h
			inc bx
			jmp .loop
		.compression:
			mov byte [es:.clValue], cl
			lodsw
			mov cx, ax
			lodsb
			mov dl, al
		.compression_loop:
			cmp bx, word [es:.SpriteX]
			je .compression_next_row
			push cx
			mov cl, byte [es:.clValue]
			push 0x81
			int 80h
			pop cx
			inc bx
			loop .compression_loop
			mov cl, byte [es:.clValue]
			jmp .loop
		.compression_next_row:
			mov bx, word [es:.OriginalBX]
			inc byte [es:.clValue]
			mov al, byte [es:.clValue]
			cmp byte [es:.SpriteY], al
			je .done
			push cx
			mov cl, byte [es:.clValue]
			push 0x81
			int 80h
			pop cx
			inc bx
			loop .compression_loop
			mov cl, byte [es:.clValue]
			jmp .loop
	.done:
		pop es
		pop si
		pop dx
		pop cx
		pop bx
		pop ax
		iret

	.clValue		db	0x00
	.OriginalBX		dw	0x0000
	.SpriteX		dw	0x0000
	.SpriteY		db	0x00
; --------------------------------------------------------------------------------------------
; Show text mode cursor.
; --------------------------------------------------------------------------------------------
; IN:  None
; OUT: None
; Special: Registers preserved.
enable_cursor:
	push ax
	push ds
	mov ax, KernelSpace
	mov ds, ax
	call draw_cursor
	mov byte [CursorStatus], 0x01
	pop ds
	pop ax
	iret

; --------------------------------------------------------------------------------------------
; Enter graphics mode.
; --------------------------------------------------------------------------------------------
; IN:  None
; OUT: None
; Special: Registers preserved.
enter_graphics_mode:
    push ax
	mov al, 0x13
    mov ah,00                		; Subfunction 0
    int 10h                   		; Call BIOS
	push 0x85						; Clear screen
	int 80h
    pop ax
    iret

; --------------------------------------------------------------------------------------------
; Exit graphics mode entering text mode.
; --------------------------------------------------------------------------------------------
; IN:  None
; OUT: None
; Special: Registers preserved.
exit_graphics_mode:
    push ax
	push bx
	push cx
	push dx
	mov al, 0x03
    mov ah,00                		; Subfunction 0
    int 10h                    		; Call BIOS
	mov ax, 0x1003
	mov bl, 0x00
	xor bh, bh
	int 0x10						; Disable blinking with BIOS
	mov dh, 24
	mov dl, 80
	mov bh, 0x00
	mov ah, 0x02
	int 0x10						; Disable BIOS cursor
	mov ah, 0x02
	mov al, 0x70
	push 0x11
	int 80h						; Set palette and reset screen
	push 0x0A
	int 80h
	pop dx
	pop cx
	pop bx
    pop ax
    iret
	
; --------------------------------------------------------------------------------------------
; Convert FAT time & date to usable. Returns seconds, minutes, hours, days, years, months.
; --------------------------------------------------------------------------------------------
; IN:  AX --> FAT time
;      BX --> FAT date
; OUT: AL --> Seconds
;      AH --> Minutes
;      BL --> Hours
;      BH --> Days
;      CL --> Months
;      DX --> Years
fat_time_to_integer:
	push ds
	mov dx, KernelSpace
	mov ds, dx
	mov word [.RawTime], ax
	mov word [.RawDate], bx
	mov ax, word [.RawTime]
	and ax, 0000000000011111b		; Extract seconds/2
	mov bx, 2						; Seconds*2
	mul bx
	mov byte [.Seconds], al		; Save
	mov ax, word [.RawTime]
	and ax, 0000011111100000b		; Extract minutes
	shr ax, 5						; Adjust it
	mov byte [.Minutes], al		; Save
	mov ax, word [.RawTime]
	and ax, 1111100000000000b		; Extract hours
	shr ax, 11						; Adjust it
	mov byte [.Hours], al			; Save
	mov ax, word [.RawDate]
	and ax, 0000000000011111b		; Extract day
	mov byte [.Day], al			; Save
	mov ax, word [.RawDate]
	and ax, 0000000111100000b		; Extract month
	shr ax, 5						; Adjust it
	mov byte [.Month], al			; Save
	mov ax, word [.RawDate]
	and ax, 1111111000000000b		; Extract year-1980
	shr ax, 9						; Adjust it
	add ax, 1980					; Add 1980
	mov word [.Year], ax			; Save
	mov al, byte [.Seconds]
	mov ah, byte [.Minutes]
	mov bl, byte [.Hours]
	mov bh, byte [.Day]
	mov cl, byte [.Month]
	mov dx, word [.Year]
	pop ds
	iret

	.RawDate	dw 0x0000
	.RawTime	dw 0x0000
	.Seconds	db 0x00
	.Minutes	db 0x00
	.Hours		db 0x00
	.Day		db 0x00
	.Month		db 0x00
	.Year		dw 0x0000

; --------------------------------------------------------------------------------------------
; Read selected byte from floppy drive.
; --------------------------------------------------------------------------------------------
; IN:  EBX --> Byte address
;      DL  --> Drive number
; OUT: AL  --> Byte read
; Special: Registers preserved.
floppy_read_byte:
	push ebx
	push ecx
	push edx
	push ds
	push es
	mov cx, KernelSpace				; Prepare DS and ES
	mov ds, cx
	mov es, cx
	push eax						; Save EAX to save the other parts of the register
	push dx							; Save target drive for later
	xor edx, edx					; Prepare EDX
	mov eax, ebx					; Put the address in EAX
	mov ebx, 512					; Prepare to divide by 512
	div ebx							; Divide
	mov word [.target_sector], ax
	mov word [.target_offset], dx
	pop dx
	cmp byte [CacheStatus], 0xFF
	jne .cache_miss
	cmp ax, word [SectorInCache]
	jne .cache_miss
	cmp dl, byte [DriveInCache]
	jne .cache_miss
	.cache_hit:
		mov bx, word [.target_offset]
		pop eax						; Restore the rest of EAX
		add bx, DiskCache
		mov al, byte [bx]
		jmp .done
	.cache_miss:
		mov byte [CacheStatus], 0xFF	; Flag cache as enabled
		mov ax, word [.target_sector]
		mov bx, DiskCache				; Target buffer is cache
		mov cx, 1						; Read 1 sector
		push 0x23
		int 80h						; Read sector
		mov word [SectorInCache], ax	; Set cache metadata
		mov byte [DriveInCache], dl
		jmp .cache_hit					; Now we can use the cache
	.done:
		pop es
		pop ds
		pop edx
		pop ecx
		pop ebx
		iret

	.target_sector		dw	0x0000
	.target_offset		dw	0x0000

; --------------------------------------------------------------------------------------------
; Read selected dword from floppy drive.
; --------------------------------------------------------------------------------------------
; IN:  EBX --> Dword address
;      DL  --> Drive number
; OUT: EAX --> Dword read
; Special: Registers preserved.
floppy_read_dword:
	push ebx
	push cx								; Save regs
	add ebx, 3							; Read last to first byte, since it's little endian
	mov cx, 4							; Loop 4 times
	.loop:
		shl eax, 8						; Rotate EAX left
		push 0x24						; Read byte call
		int 80h
		dec ebx							; Next byte
		loop .loop						; Loop
	pop cx								; Restore regs
	pop ebx
	iret

; --------------------------------------------------------------------------------------------
; Read multiple sectors from floppy. LBA adressed.
; --------------------------------------------------------------------------------------------
; IN:  AX = LBA starting sector
; 	   DL = Drive number
;      ES = Buffer segment
; 	   EBX = Buffer offset
; 	   CX = Sectors count
; OUT: None
; Special: Registers preserved.
floppy_read_sectors:
	push eax								; Save GPRs
	push ebx
	push ecx
	push edx
	push esi
	push edi
	push ds
	mov si, KernelSpace
	mov ds, si
	.loop:
		push es
		push ebx
		mov si, KernelSpace
		mov es, si
		mov byte [CacheStatus], 0xFF
		mov byte [DriveInCache], dl
		mov word [SectorInCache], ax
		mov bx, DiskCache
		call floppy_read_sector				; Read sector
		pop ebx
		pop es
		jc .done							; If carry, exit routine
		mov edi, ebx
		mov esi, DiskCache
		push ecx
		mov ecx, 512
		a32 o32 rep movsb
		pop ecx
		inc ax								; Increment sector
		add ebx, 512						; Increment buffer
		loop .loop							; Loop!
	.done:
		pop ds
		pop edi
		pop esi
		pop edx
		pop ecx								; Restore GPRs
		pop ebx
		pop eax
		iret								; Exit routine
		
; --------------------------------------------------------------------------------------------
; Read selected word from floppy drive.
; --------------------------------------------------------------------------------------------
; IN:  EBX --> Word address
;      DL  --> Drive number
; OUT: AX  --> Word read
; Special: Registers preserved.
floppy_read_word:
	inc ebx									; Read last to first byte, since it's little endian
	push 0x24								; Read byte call
	int 80h
	mov ah, al								; Put the higher byte in AH
	dec ebx									; Next byte
	push 0x24								; Read byte call
	int 80h
	iret

; --------------------------------------------------------------------------------------------
; Write byte to selected address in floppy.
; --------------------------------------------------------------------------------------------
; IN:  EBX --> Byte address
;      AL  --> Byte to write
;      DL  --> Drive number
; OUT: 		   None
; Special: Registers preserved.
floppy_write_byte:
	push eax
	push ebx
	push ecx
	push edx
	push ds
	push es
	mov cx, KernelSpace						; Prepare DS and ES
	mov ds, cx
	mov es, cx
	mov byte [.ByteToWrite], al			; Save byte to write for later
	push dx									; Save target drive for later
	xor edx, edx							; Prepare EDX
	mov eax, ebx							; Put the address in EAX
	mov ebx, 512							; Prepare to divide by 512
	div ebx									; Divide
	mov word [.target_sector], ax
	mov word [.target_offset], dx
	pop dx
	cmp byte [CacheStatus], 0xFF
	jne .cache_miss
	cmp ax, word [SectorInCache]
	jne .cache_miss
	cmp dl, byte [DriveInCache]
	jne .cache_miss
	.cache_hit:
		mov bx, word [.target_offset]
		add bx, DiskCache
		mov al, byte [.ByteToWrite]
		mov byte [bx], al
		jmp .write_cache
	.cache_miss:
		mov byte [CacheStatus], 0xFF		; Flag cache as enabled
		mov ax, word [.target_sector]
		mov bx, DiskCache					; Target buffer is cache
		mov cx, 1							; Read 1 sector
		push 0x23
		int 80h							; Read sector
		mov word [SectorInCache], ax		; Set cache metadata
		mov byte [DriveInCache], dl
		jmp .cache_hit						; Now we can use the cache
	.write_cache:
		mov ax, [SectorInCache]
		mov dl, [DriveInCache]
		mov bx, DiskCache
		mov cx, 1
		push 0x31
		int 80h
		pop es
		pop ds
		pop edx
		pop ecx
		pop ebx
		pop eax
		iret

	.ByteToWrite		db	0x00
	.target_sector		dw	0x0000
	.target_offset		dw	0x0000

; --------------------------------------------------------------------------------------------
; Write dword to selected address on floppy.
; --------------------------------------------------------------------------------------------
; IN:  EBX --> Byte address
;      EAX --> Byte to write
;      DL  --> Drive number
; OUT: 		   None
; Special: Registers preserved.
floppy_write_dword:
	push eax
	push ebx
	push cx									; Save regs
	mov cx, 4								; Loop 4 times
	.loop:
		push 0x32							; Write byte call
		int 80h
		shr eax, 8							; Rotate EAX right
		inc ebx								; Next byte
		loop .loop							; Loop
	pop cx									; Restore regs
	pop ebx
	pop eax
	iret

; --------------------------------------------------------------------------------------------
; Write sectors to selected LBA on floppy.
; --------------------------------------------------------------------------------------------
; IN:  AX --> LBA starting sector
;      DL --> Drive number
;      ES --> Buffer segment
;      BX --> Buffer offset
;      CX --> Sectors count
; OUT: 		   None
; Special: Registers preserved.
floppy_write_sectors:
	push ax									; Save GPRs
	push bx
	push cx
	push dx
	.loop:
	call floppy_write_sector				; Write sector
	jc .done								; If carry, exit routine
	inc ax									; Increment sector
	add bx, 512								; Add 512 to the buffer
	loop .loop								; Loop
	.done:
	pop dx
	pop cx									; Restore GPRs
	pop bx
	pop ax
	iret									; Exit routine

; --------------------------------------------------------------------------------------------
; Write word to location on a floppy.
; --------------------------------------------------------------------------------------------
; IN:  AX  --> Word to write
;      EBX --> Word address
;      DL  --> Drive number
; OUT: 		   None
; Special: Registers preserved.
floppy_write_word:
	push ax
	xchg al, ah
	inc ebx									; Write last to first byte, since it's little endian
	push 0x32								; Write byte call
	int 80h
	xchg al, ah
	dec ebx									; Next byte
	push 0x32								; Write byte call
	int 80h
	pop ax
	iret
	
; --------------------------------------------------------------------------------------------
; Read character
; --------------------------------------------------------------------------------------------
; IN: 		  None
; OUT: AL --> ASCII char
; Special: Registers preserved.

get_char:
	push ds
	push ax
	mov ax, KernelSpace
	mov ds, ax
	xor ax, ax
	int 0x16
	mov byte [.ReturnASCII], al
	pop ax
	mov al, byte [.ReturnASCII]
	pop ds
	iret
	.ReturnASCII		db	0x00
	
; --------------------------------------------------------------------------------------------
; Return the current path in ES:EDI
; --------------------------------------------------------------------------------------------
; IN: Inherited
; OUT: None
; Special: Registers preserved.
get_current_dir:
	push esi
	push ds
	mov si, KernelSpace
	mov ds, si
	mov esi, CurrentDirectoryPath
	push 0x27
	int 80h													; Happy birthday, MS-DOS!
	pop ds
	pop esi
	iret

; --------------------------------------------------------------------------------------------
; Return the current drive.
; --------------------------------------------------------------------------------------------
; IN: 		  Inherited
; OUT: DL --> Drive number
; Special: Registers preserved.
get_current_drive:
	push ax
	push ds
	mov ax, KernelSpace
	mov ds, ax
	mov dl, byte [CurrentDrive]
	pop ds
	pop ax
	iret
; --------------------------------------------------------------------------------------------
; Return the current pallette.									; Now, sing me happy birthday!
; --------------------------------------------------------------------------------------------
; IN:  None
; OUT: AH --> Char palette
;      AL --> Cursor palette
; Special: Registers preserved

get_current_palette:											; RIP Dennis Ritchie
	push ds
	mov ax, KernelSpace
	mov ds, ax
	mov ah, byte [CharAttributes]
	mov al, byte [CursorAttributes]
	pop ds
	iret
	
; --------------------------------------------------------------------------------------------
; Return position of text mode cursor.
; --------------------------------------------------------------------------------------------
; IN:  None
; OUT: AH = cursor X
;      AL = cursor Y
; Special: Registers preserved

get_cursor_position:
	push bx
	push cx
	push dx
	push ds
	mov ax, KernelSpace
	mov ds, ax
	mov ax, word [CursorLocation]
	mov bx, 160													; Divide AX / 160
	xor dx, dx
	div bx
	xor ah, ah													; Clear AH
	push ax														; Push result, for now
	mov ax, dx													; Load MOD
	mov bx, 2													; Divide MOD / 2
	xor dx, dx
	div bx
	mov dx, ax													; Result in DX
	pop ax														; Restore AX
	mov ah, dl													; Move AH
	pop ds
	pop dx
	pop cx
	pop bx
	iret

; --------------------------------------------------------------------------------------------
; Return kernel version.
; --------------------------------------------------------------------------------------------
; IN:  None
; OUT: ES:EDI --> version number string in ASCII
; Special: Registers preserved.
get_version_number:
	push esi
	push ds
	mov si, KernelSpace
	mov ds, si
	mov esi, Version
	push 0x27
	int 80h
	pop ds
	pop esi
	iret

; --------------------------------------------------------------------------------------------
; Clear text mode screen to match new pallette.
; --------------------------------------------------------------------------------------------
; IN:  None
; OUT: None
; Special: Registers preserved.
initialise_screen:
	push ax
	push cx
	push di
	push ds
	push es
	mov ax, 0xB800
	mov es, ax
	mov ax, KernelSpace
	mov ds, ax
	xor di, di													; Point to video memory
	mov ah, byte [CursorAttributes]							; Get cursor attributes
	mov al, ' '
	stosw
	mov ah, byte [CharAttributes]								; Get char attributes
	mov al, ' '
	mov cx, 0x07CF
	rep stosw
	mov word [CursorLocation], 0x0000
	mov byte [CursorStatus], 0x01
	pop es
	pop ds
	pop di
	pop cx
	pop ax
	iret
	
; --------------------------------------------------------------------------------------------
; Read 32-bit integer and return it in EAX.
; --------------------------------------------------------------------------------------------
; IN:  None
; OUT: Inherited
; Special: Registers preserved.

input_integer:
	push ebx
	push ecx
	push edx
	push si
	push di
	push ds
	push es

	mov ax, KernelSpace
	mov ds, ax
	mov es, ax

	xor cx, cx
	xor dx, dx
	mov di, .buffer
	.loop:
		push 0x1C
		int 80h
		cmp al, 0x08
		je .backspace
		cmp al, 0x0D
		je .enter
		cmp al, '0'
		jl .loop
		cmp al, '9'
		jg .loop
		cmp cl, 10
		je .loop
		inc cl
		push 0x01
		int 80h
		sub al, '0'
		stosb
		jmp .loop
	.backspace:
		cmp cl, 0
		je .loop
		mov al, 0x08
		push 0x01
		int 80h
		dec di
		dec cl
		jmp .loop
	.enter:
		mov si, .buffer
		xor eax, eax
	.ascii_to_integer:
		cmp cl, 0x00
		je .done
		mov edx, 10
		mul edx
		xor edx, edx
		mov dl, byte [si]
		inc si
		add eax, edx
		dec cl
		jmp .ascii_to_integer
	.done:
		pop es
		pop ds
		pop di
		pop si
		pop edx
		pop ecx
		pop ebx
		iret
	.buffer times 10 db 0x00

; --------------------------------------------------------------------------------------------
; Read string from keyboard.
; --------------------------------------------------------------------------------------------
; IN: EBX    --> Max string length
;     ES:EDI --> Pointer to buffer.
input_string:
	push ax
	push ebx
	push ecx
	push edi
	xor ecx, ecx
	.loop:
		push 0x1C
		int 80h
		cmp al, 0x08
		je .backspace
		cmp al, 0x0D
		je .enter
		test al, al
		jz .loop
		cmp ecx, ebx
		je .loop
		inc ecx
		push 0x01
		int 80h
		a32 o32 stosb
		jmp .loop
	.backspace:
		test ecx, ecx
		jz .loop
		mov al, 0x08
		push 0x01
		int 80h
		dec edi
		dec ecx
		jmp .loop
	.enter:
		xor al, al
		a32 o32 stosb
		pop edi
		pop ecx
		pop ebx
		pop ax
		iret

; --------------------------------------------------------------------------------------------
; Invalidate cache.
; --------------------------------------------------------------------------------------------
; IN:  None
; OUT: None
; Special: Registers preserved.
invalid_cache:
	push ax
	push ds
	mov ax, KernelSpace
	mov ds, ax
	mov byte [CacheStatus], 0x00
	pop ds
	pop ax
	iret

; --------------------------------------------------------------------------------------------
; Load new working directory.
; --------------------------------------------------------------------------------------------
; IN:  DS:SI --> Directory name
; OUT: DL    --> 0xFF if failure, cleared on success
; Special: Registers preserved.
load_dir:
	push ax
	push bx
	push cx
	push si
	push di
	push es
	push dx
	mov ax, KernelSpace
	mov es, ax
	mov word [es:.DirName], si
	mov dl, 0x01
	push 0x2B											; Get starting cluster and size (directory)
	int 80h
	test al, al
	jnz .failure										; Check for success
	mov ax, si
	mov word [es:CurrentDirectoryCluster], ax
	push 0x13											; Get current drive
	int 80h
	call erase_dir_cache								; Erase the directory cache
	mov bx, CurrentDirectoryCache						; Load in the current directory cache
	test ax, ax										; Check if chain points to 0x0000 (root)
	jz .load_root
	call fat_load_chain									; Load chain into the buffer
	jmp .success
	.load_root:
		call fat_load_root
		jmp .success
	.failure:
		pop dx
		xor dl, dl
		not dl
		jmp .done
	.success:
		mov si, word [es:.DirName]
		mov di, CurrentDirectoryPath						; Save current path
		call path_converter
		pop dx
		xor dl, dl
	.done:
		pop es
		pop di
		pop si
		pop cx
		pop bx
		pop ax
		iret
	.DirName	dw	0x0000

; --------------------------------------------------------------------------------------------
; Load file from current working directory
; --------------------------------------------------------------------------------------------
; IN:  ES:BX --> Target buffer (segment:offset)
;      DS:SI --> Filename (has to be in readable form, not the FAT one)
; OUT: ecx   --> File size (B)
;      dl    --> 0xFF if failure, cleared on success
; Special: Registers preserved.
load_file:
	push ax
	push bx
	push si
	push di
	push es
	push gs
	push dx
	mov cx, es
	mov ax, KernelSpace
	mov gs, ax
	mov word [gs:.TargetSegment], cx
	mov word [gs:.TargetBuffer], bx
	xor dl, dl
	push 0x2B												; Get starting cluster and size (file)
	int 80h
	test al, al
	jnz .failure											; Check for success
	mov ax, si
	test ax, ax
	jz .success												; Check for empty file
	push cx
	mov cx, word [gs:.TargetSegment]
	mov es, cx
	mov bx, word [gs:.TargetBuffer]
	push 0x13												; Get current drive
	int 80h
	call fat_load_chain										; Load chain into the buffer
	pop cx
	jmp .success
	.failure:
		pop dx
		xor dl, dl
		not dl
		jmp .done
	.success:
		pop dx
		xor dl, dl
	.done:
		pop gs
		pop es
		pop di
		pop si
		pop bx
		pop ax
		iret

	.TargetSegment dw 0x0000
	.TargetBuffer  dw 0x0000

; --------------------------------------------------------------------------------------------
; Convert string to uppercase.
; --------------------------------------------------------------------------------------------
; IN:  ES:EDI - string
; OUT: Inherited
; Special: Registers preserved
lower_to_uppercase:
	push ax
	push edi
	.loop:
		mov al, byte [es:edi]	; Byte from EDI
		cmp al, 0x60
		jg .convert
		cmp al, 0x00
		je .done
	.no_convert:
		inc edi
		jmp .loop
	.convert:
		cmp al, 0x7A
		jg .no_convert
		sub al, 0x20
		a32 o32 stosb
		jmp .loop
	.done:
		pop edi
		pop ax
		iret

; --------------------------------------------------------------------------------------------
; Print out newline.
; --------------------------------------------------------------------------------------------
; IN:  None
; OUT: None
; Special: Registers preserved.
new_line:
	push ax
	mov al, 0x0A
	push 0x01
	int 80h
	pop ax
	iret
	
; --------------------------------------------------------------------------------------------
; Pause execution until keypress
; --------------------------------------------------------------------------------------------
; IN:  None
; OUT: None
; Special: Registers preserved.
pause:
	push ax
	push 0x1C
	int 80h
	pop ax
	iret
	
; --------------------------------------------------------------------------------------------
; Read file metadata
; --------------------------------------------------------------------------------------------
; IN:  DS:SI --> Relative path
;      DL    --> 1 for dir, 0 for file
; OUT: SI   --> starting cluster, 0x0000 if not found
;      AL   --> 0x00 if success, 0xFF if not found
;      BX   --> raw FAT time
;      ECX  --> file size
;      DX   --> raw FAT date

ping_file:
	push di
	push ds
	push es
	mov ax, KernelSpace
	mov es, ax
	mov byte [es:.DirectoryFlag], dl
	lodsb
	cmp al, '/'
	je .load_root
	dec si
	.loop:
		mov di, .FileName
		mov cx, 13
	.get_file_name:
		lodsb
		cmp al, '/'
		je .fetch_entry
		stosb
		test al, al
		jz .fetch_target_entry
		loop .get_file_name
	.fetch_entry:
		xor al, al
		stosb
		mov dl, 1										; Looking for a dir
		push si
		push ds
		mov ax, KernelSpace
		mov ds, ax
		mov si, .FileName
		call fat_get_metadata
		mov word [.Cluster], si
		pop ds
		pop si
		cmp al, 0xFF
		je .restore
		call erase_dir_cache
		mov ax, word [es:.Cluster]
		mov bx, CurrentDirectoryCache
		push 0x13
		int 80h
		test ax, ax
		jz .load_root
		call fat_load_chain
		jmp .loop
	.load_root:
		call fat_load_root
		jmp .loop
	.fetch_target_entry:
		mov dl, byte [es:.DirectoryFlag]
		push ax
		mov ax, KernelSpace
		mov ds, ax
		pop ax
		mov si, .FileName
		call fat_get_metadata
	.restore:
		push ax
		push bx
		push cx
		push dx
		push si
		push di
		push 0x13
		int 80h
		call erase_dir_cache
		mov ax, word [CurrentDirectoryCluster]
		test ax, ax
		jz .load_root_end
		mov bx, CurrentDirectoryCache
		call fat_load_chain
		jmp .load_dir_done
	.load_root_end:
		call fat_load_root
	.load_dir_done:
		pop di
		pop si
		pop dx
		pop cx
		pop bx
		pop ax
	.done:
		pop es
		pop ds
		pop di
		iret

	.DirectoryFlag		db	0x00
	.Cluster			dw	0x0000
	.FileName			times	14	db 0x00

; --------------------------------------------------------------------------------------------
; Play sound data contained in buffer
; --------------------------------------------------------------------------------------------
; IN:  DS:ESI --> Start address of sound data
;      DL     --> Repeat flag
; OUT: None
play_music:
	push cx
	push es
	mov cx, KernelSpace
	mov es, cx
	mov cx, ds
	mov word [es:MusicSegment], cx
	mov dword [es:MusicStartAddress], esi
	mov dword [es:MusicCurrentNote], esi
	mov byte [es:MusicRepeatFlag], dl
	mov cx, word [ds:esi+1]
	push 0x22
	int 80h
	mov byte [es:MusicPlayingFlag], 1
	pop es
	pop cx
	iret

; --------------------------------------------------------------------------------------------
; Print 32-bit integer on screen.
; --------------------------------------------------------------------------------------------
; IN:  EAX    --> Number
;      DL     --> If 1, then right align. If 0, not.
;      CL     --> Minimum number of digits (pad with ASCII 0x30)
; OUT: None
print_integer:
	push eax
	push ebx
	push ecx
	push edx
	push di
	push si
	push ds
	mov bx, KernelSpace
	mov ds, bx											; Point to kernel space
	xor ch, ch
	cmp dl, 1
	je .right_align
	mov dl, 0xFF
	jmp .main
	.right_align:
		mov dl, ' '
	.main:
		push cx
	.clear_buffer:
		mov cx, 10
		mov di, .buffer
		.clear_buffer_loop:
			mov byte [di], dl
			inc di
		loop .clear_buffer_loop
	pop cx
	test cl, cl
	jz .calculate
	mov di, (.buffer+9)
	.pad:
		mov byte [di], '0'
		dec di
		loop .pad
	.calculate:
		mov di, (.buffer+9)
		mov byte [di], '0'
	.loop:
		xor edx, edx
		mov ebx, 10
		cmp eax, 0x00
		je .done
		div ebx
		add dl, '0'
		mov byte [di], dl
		dec di
		jmp .loop
	.done:
		mov si, .buffer
		push 0x02
		int 80h
		pop ds
		pop si
		pop di
		pop edx
		pop ecx
		pop ebx
		pop eax
		iret
	.buffer times 10 db 0x00
					 db 0x00
; --------------------------------------------------------------------------------------------
; Print 32-bit integer on screen. Base 16.
; --------------------------------------------------------------------------------------------
; IN:  EAX    --> Number
;      DL     --> If 1, then right align. If 0, not.
; OUT: None
print_integer_hex:
	push eax
	push ebx
	push ecx
	push edx
	push di
	push si
	push ds
	mov bx, KernelSpace
	mov ds, bx										; Point to kernel space
	cmp dl, 1
	je .right_align
	mov dl, 0xFF
	jmp .clear_buffer
	.right_align:
		mov dl, ' '
	.clear_buffer:
		mov cx, 8
		mov di, .buffer
		.clear_buffer_loop:
			mov byte [di], dl
			inc di
		loop .clear_buffer_loop
		jmp .calculate
	.calculate:
		mov di, (.buffer+7)
		mov byte [di], '0'
	.loop:
		xor edx, edx
		mov ebx, 16
		cmp eax, 0x00
		je .done
		div ebx
		add dl, '0'
		cmp dl, '9'
		jg .adjust_for_ascii
	.adjusted:
		mov byte [di], dl
		dec di
		jmp .loop
	.adjust_for_ascii:
		add dl, 7
		jmp .adjusted
	.done:
		mov si, .buffer
		push 0x02
		int 80h
		pop ds
		pop si
		pop di
		pop edx
		pop ecx
		pop ebx
		pop eax
		iret
	.buffer times 8 db 0x00
					db 0x00
; --------------------------------------------------------------------------------------------
; Print null terminated string
; --------------------------------------------------------------------------------------------
; IN:  ESI --> String
; OUT: None
print_string:
	push ax
	push esi
	.loop:
		a32 o32 lodsb									; Byte from ESI
		test al, al										; Is 0x00?
		jz .done										; If yes, done
		push 0x01
		int 80h
		jmp .loop										; Loop
	.done:
		pop esi
		pop ax
		iret
		
; --------------------------------------------------------------------------------------------
; Push frame
; --------------------------------------------------------------------------------------------
; IN:  Inherited
; OUT: Inherited
push_frame:
	push ds
	push es
	push esi
	push edi
	push ecx
	xor cx, cx
	mov ds, cx
	mov cx, 0xA000
	mov es, cx
	mov esi, FrameBuffer
	xor edi, edi
	xor ecx, ecx
	not cx
	a32 o32 rep movsb
	pop ecx
	pop edi
	pop esi
	pop es
	pop ds
	iret
	
; --------------------------------------------------------------------------------------------
; Print character
; --------------------------------------------------------------------------------------------
; IN:  AL --> Input character
;      Inherited
; OUT: Inherited
put_char:
	push ax
	push bx
	push cx
	push dx
	push es
	push fs
	cmp al, 0xFF								; Is 0xFF (null)?
	je .done									; Ignore
	test al, al									; Is 0x00 (null)?
	jz .done									; Ignore
	mov bx, 0xB800								; Point ES to video memory
	mov es, bx
	mov bx, KernelSpace							; Point FS to kernel space
	mov fs, bx
	cmp al, 0x0A								; Is 0x0A?
	je .next_line
	cmp al, 0x08								; Is 0x08?
	je .backspace
	cmp al, 0x09								; Is 0x09?
	je .tab
	mov bx, word [fs:CursorLocation]
	inc bx
	mov ah, byte [fs:CharAttributes]
	mov byte [es:bx], ah						; Attributes first to avoid 'ghosting'
	dec bx
	mov byte [es:bx], al						; Print character
	inc bx
	inc bx
	cmp bx, 0x0FA0								; End of video memory?
	je .scroll_screen_down
	mov word [fs:CursorLocation], bx
	cmp byte [fs:CursorStatus], 1				; Check if cursor is enabled
	jne .done
	call draw_cursor							; Draw cursor
	jmp .done
	.tab:
		push 0x0D
		int 80h									; Get cursor coordinates
		xor al, al
		shr ax, 8
		mov bx, 8									; Divide X by 8 to get the cell number
		xor dx, dx
		div bx
		mov cx, 8
		sub cx, dx									; Retrieve %
		mov al, ' '									; Fill space with spaces
	.add_tab_loop:
		push 0x01
		int 80h
		loop .add_tab_loop
		jmp .done
	.next_line:
		push 0x0D
		int 80h									; Get cursor coordinates
		xor ah, ah									; Set X to 0
		cmp al, 24									; Last line?
		je .scroll_screen_down						; Scroll screen
		inc al										; Y + 1
		push 0x0E
		int 80h									; Move cursor!
		jmp .done									; Done
	.backspace:
		sub word [fs:CursorLocation], 2
		mov al, ' '
		push 0x01
		int 80h
		cmp byte [fs:CursorStatus], 1
		jne .back_no_cursor
		call clear_cursor
	.back_no_cursor:
		sub word [fs:CursorLocation], 2
		cmp byte [fs:CursorStatus], 1
		jne .done
		call draw_cursor
		jmp .done
	.scroll_screen_down:
		call clear_cursor							; Destroy cursor
		mov bx, 160									; Next line
	.scroll_down_loop:
	cmp bx, 0x0FA0								; Last char?
		je .clear_line
		mov ax, word [es:bx]						; Get word
		mov word [es:bx-160], ax					; Copy to previous line
		inc bx
		inc bx
		jmp .scroll_down_loop
	.clear_line:
		sub bx, 160
		mov word [fs:CursorLocation], bx			; New cursor location
		mov al, byte [fs:CharAttributes]
	.clear_line_loop:
		cmp bx, 0x0FA0
		je .clear_line_done
		mov byte [es:bx], ' '
		inc bx
		mov byte [es:bx], al
		inc bx
		jmp .clear_line_loop
	.clear_line_done:
		cmp byte [fs:CursorStatus], 1
		jne .done
		call draw_cursor							; Restore cursor
	.done:
		pop fs
		pop es
		pop dx
		pop cx
		pop bx
		pop ax
		iret
	
	.tab_loop_flag					db 0x00

; --------------------------------------------------------------------------------------------
; Set current drive
; --------------------------------------------------------------------------------------------
; IN:  DL --> Drive number
; OUT: None
; Special: Registers preserved
set_current_drive:
	push ax
	push ds
	mov ax, KernelSpace
	mov ds, ax
	mov byte [CurrentDrive], dl						; Set up the current drive variable
	mov word [CurrentDirectoryPath], 0x002F			; Set the current directory path
	call fat_load_root								; Load the root directory
	pop ds
	pop ax
	iret

; --------------------------------------------------------------------------------------------
; Set text mode cursor position.
; --------------------------------------------------------------------------------------------
; IN:  AH --> cursor X
;      AL --> cursor Y
; OUT: None
; Special: Registers preserved.

set_cursor_position:
	push ax
	push bx
	push cx
	push dx
	push ds
	mov bx, KernelSpace
	mov ds, bx
	cmp byte [CursorStatus], 0x01
	jne .main
	call clear_cursor		; Clear cursor
	.main:
		push ax						; Save AX
		xor ah, ah					; Clear high 8
		mov bx, 160					; Multiply Y * 160
		mul bx
		mov word [CursorLocation], ax	; Save for now
		pop ax						; Restore AX
		xor al, al					; Clear low 8
		shr ax, 8					; Shift right 8
		add ax, ax					; Multiply X * 2
		add word [CursorLocation], ax	; Add
		cmp byte [CursorStatus], 0x01
		jne .done
		call draw_cursor		; Draw cursor back
	.done:
		pop ds
		pop dx
		pop cx
		pop bx
		pop ax
		iret
		
; --------------------------------------------------------------------------------------------
; Set new text mode palette.
; --------------------------------------------------------------------------------------------
; IN:  AH --> character attributes
;      AL --> cursor attributes
; OUT: None
set_palette:
	push bx
	push ds
	mov bx, KernelSpace
	mov ds, bx
	mov byte [CharAttributes], ah
	mov byte [CursorAttributes], al
	cmp byte [CursorStatus], 0x01
	jne .done
	call draw_cursor
	.done:
		pop ds
		pop bx
		iret

; --------------------------------------------------------------------------------------------
; Sleep for specified amount of ticks
; --------------------------------------------------------------------------------------------
; IN:  ECX: Number of ticks of inactivity
; OUT: None
sleep:
	test ecx, ecx
	jz .quick_exit
	sti
	push ax
	push ecx
	push ds
	mov ax, KernelSpace
	mov ds, ax
	add ecx, dword [ClockTicks]
	.loop:
		cmp dword [ClockTicks], ecx
		je .done
		jmp .loop
	.done:
		pop ds
		pop ecx
		pop ax
		cli
	.quick_exit:
		iret

; --------------------------------------------------------------------------------------------
; Start new process.
; --------------------------------------------------------------------------------------------
; IN:  DS:ESI --> Name of new program
;      DS:EDI --> Command line switches
; OUT: EAX    --> 0xFFFFFFFF if error / not found
start_new_program:
	push ebx							; Save general purpose registers
	push ecx							; (doesn't save AX because it's the exit code register)
	push edx
	push esi
	push edi
	push ebp
	push ds								; Save segment registers
	push es
	push fs
	push gs
	mov ax, KernelSpace					; Point GS to kernel
	mov gs, ax
	push dword [gs:TopMemory]			; Save extended memory top
	push word [gs:TopSegment]			; Save last top of memory in stack
	push word [gs:StackSegment]			; Save last stack segment in stack
	push dword [gs:StackPointer]		; Save last stack pointer in stack
	mov ax, ss
	mov word [gs:StackSegment], ax		; Save new stack segment in RAM
	mov dword [gs:StackPointer], esp	; Save new stack pointer in RAM
	mov ax, word [gs:TopSegment]		; Prepare to load program
	mov es, ax
	mov ebx, 0x0100						; Load at offset es:00000100
	push 0x12
	int 80h							; Load program
	cmp dl, 0xFF
	je .drop_out						; If file not found, drop out
	cmp ecx, 0x0000FFFF					; Test if program is too big (> 64KB)
	jg .drop_out						; If it is, drop out
	call check_bin_extension
	jc .drop_out
	add cx, 0x0100						; Add the kernel reserved space to the file size
	shr cx, 4							; Shift CX right by 4 bits to get size in 16 bytes blocks
	inc cx								; Adjust CX
	add word [gs:TopSegment], cx		; Reduce available memory
	mov esi, edi						; DS:EDI in DS:ESI
	xor edi, edi						; Prepare ES:EDI to the string copy
	push 0x27							; Copy string
	int 80h
	mov cx, word [gs:TopSegment]		; Prepare the stack segment
	mov ss, cx
	mov esp, 0x1f0						; Prepare the stack pointer
	sti									; Restore interrupts
	add word [gs:TopSegment], 0x20		; Allocate 512 more bytes for the stack
	mov ds, ax							; Prepare all segment registers (AX was already correctly loaded)
	mov fs, ax
	mov gs, ax
	push ax								; Prepare stack for retf
	push 0x0100
	xor eax, eax						; Flush all GPRs
	xor ebx, ebx
	xor ecx, ecx
	xor edx, edx
	xor esi, esi
	xor edi, edi
	xor ebp, ebp
	retf								; Retf to the program
	.drop_out:
		mov eax, 0xFFFFFFFF				; Set EAX to 0xFFFFFFFF
		push 0x00
		int 80h						; Terminate process instantly

; --------------------------------------------------------------------------------------------
; Stop beeping.
; --------------------------------------------------------------------------------------------
; IN:  None
; OUT: None
; Special: Registers preserved.
stop_beep:
	push ax
	in al, 0x61							; Disconnect speaker from timer 2
	and al, 11111100b
	out 0x61, al
	pop ax
	iret

; --------------------------------------------------------------------------------------------
; Stop music.
; --------------------------------------------------------------------------------------------
; IN:  None
; OUT: None
; Special: Registers preserved.
stop_music:
	push cx
	push es									; Stop speaker
	mov cx, KernelSpace
	mov es, cx
	push 0x1E
	int 80h
	mov byte [es:MusicPlayingFlag], 0
	pop es
	pop cx
	iret

; --------------------------------------------------------------------------------------------
; Copy string
; --------------------------------------------------------------------------------------------
; IN:  DS:ESI --> Input
; OUT: ES:EDI --> Output, null terminated
; Special: Registers preserved.
string_copy:
	push ax
	push esi
	push edi
	.loop:
		a32 o32 lodsb
		a32 o32 stosb
		test al, al
		jz .done
		jmp .loop
	.done:
		pop edi
		pop esi
		pop ax
		iret

; --------------------------------------------------------------------------------------------
; Create pointer to end of string
; --------------------------------------------------------------------------------------------
; IN & OUT: ES:EDI --> String, the same will point to the terminator.
; Special: Registers preserved.		
string_end:
	push ax
	.loop:
		mov al, byte [es:edi]
		test al, al
		jz .done
		inc edi
		jmp .loop
	.done:
		pop ax
		iret

; --------------------------------------------------------------------------------------------
; Return length of string
; --------------------------------------------------------------------------------------------
; IN:  DS:ESI --> Input string
; OUT: ECX    --> String length
; Special: Registers preserved.				
string_length:
	push ax
	push esi
	xor ecx, ecx
	.loop:
		a32 o32 lodsb						; Byte from ESI
		test al, al							; Is 0?
		jz .done
		inc ecx
		jmp .loop
	.done:
	pop esi
	pop ax
	iret

; --------------------------------------------------------------------------------------------
; Convert string to integer
; --------------------------------------------------------------------------------------------
; IN:  ESI --> Null terminated input string
; OUT: EAX --> Output integer
string_to_integer:
	push ebx
	push ecx
	push edx
	push esi
	xor ecx, ecx							; Prepare ECX
	.loop:
		xor eax, eax						; Prepare EAX
		a32 o32 lodsb						; Get byte from string
		sub al, '0'							; Get integer value
		mov dl, byte [esi]					; Check whether this is the last byte in the string
		test dl, dl							; Is the string terminated?
		jz .done							; If yes, we're done
		add ecx, eax						; Add new value to stored result
		mov eax, 10
		mul ecx								; Multiply result by 10
		mov ecx, eax						; Store result back in ECX
		jmp .loop							; Loop
	.done:
		add ecx, eax						; Add last unit
		mov eax, ecx						; Move result into EAX
		pop esi
		pop edx
		pop ecx
		pop ebx
		iret
		
; --------------------------------------------------------------------------------------------
; When "Kill myself" is getting new meaning.
; --------------------------------------------------------------------------------------------
; IN:  EAX --> Exit code
; OUT: None
; Special: Registers preserved
terminate_process:
	mov bx, KernelSpace							; Point GS to kernel
	mov gs, bx
	mov bx, word [gs:StackSegment]				; Restore last stack
	mov ss, bx
	mov esp, dword [gs:StackPointer]
	pop dword [gs:StackPointer]					; Restore older stack in RAM
	pop word [gs:StackSegment]
	pop word [gs:TopSegment]					; Restore last top segment
	pop dword [gs:TopMemory]					; Restore last top memory
	pop gs										; Restore segment registers
	pop fs
	pop es
	pop ds
	pop ebp										; Restore GPRs
	pop edi
	pop esi
	pop edx
	pop ecx
	pop ebx
	iret										; Return to calling program

; --------------------------------------------------------------------------------------------
; Read timer data. WARNING: This routine can be problematic.
; --------------------------------------------------------------------------------------------
; IN:  None
; OUT: CH --> hours
;      CL --> minutes
;      DH --> seconds
;	   DL --> DST
timer_read:
		push ax
		.retry:
		clc										; Clear carry flag
		mov ah, 0x02							; Read RTC (Hours, Minutes, Seconds)
		int 0x1A								; RTC Call
		jc .retry								; If error, retry
		mov al, ch								; Convert hours
		call .bcd_to_integer
		mov ch, al
		mov al, cl								; Minutes
		call .bcd_to_integer
		mov cl, al
		mov al, dh								; And seconds
		call .bcd_to_integer
		mov dh, al
		pop ax
		iret
		.bcd_to_integer:
		push cx									; Save CX
		push ax									; Save AX
		and al, 11110000b						; Extract the high nibble of AL
		shr al, 4								; Shift it right by 4
		mov cl, 10								; Multiply by 10
		mul cl
		pop cx									; Restore AX in CX
		and cl, 00001111b						; Extract the low nibble
		add al, cl								; And add it back to AL
		pop cx									; Restore CX
		ret
		
; --------------------------------------------------------------------------------------------
; Lowercase string.
; --------------------------------------------------------------------------------------------
; IN:  ES:EDI --> Input string
; OUT: -||-   --> Output string
upper_to_lowercase:
	push ax
	push edi
	.loop:
		mov al, byte [es:edi]					; Byte from EDI
		cmp al, 0x40
		jg .convert
		cmp al, 0x00
		je .done
	.no_convert:
		inc edi
		jmp .loop
	.convert:
		cmp al, 0x5A
		jg .no_convert
		add al, 0x20
		a32 o32 stosb
		jmp .loop
	.done:
		pop edi
		pop ax
		iret

; --------------------------------------------------------------------------------------------
; Handle request for extended memory.
; --------------------------------------------------------------------------------------------
; IN:  EAX --> Memory to allocate (in bytes)
; OUT: ECX --> Beginning of allocated memory (flat address relative to program segment)
allocate_mem32:
	push eax
	push gs
	mov cx, KernelSpace							; Point GS to kernel
	mov gs, cx
	mov ecx, dword [gs:TopMemory]				; Retrieve current top of used memory
	add dword [gs:TopMemory], eax				; Allocate memory
	xor eax, eax
	mov ax, ds
	shl eax, 4
	sub ecx, eax
	pop gs
	pop eax
	iret
	
; --------------------------------------------------------------------------------------------
; Handle request for memory.
; --------------------------------------------------------------------------------------------
; IN: AX  --> Memory to allocate (in bytes)
; OUT: CX --> Allocated segment (for data segment registers)

allocate_memory:
	push ax
	push gs
	mov cx, KernelSpace							; Point GS to kernel
	mov gs, cx
	mov cx, word [gs:TopSegment]				; Retrieve current top of used memory
	shr ax, 4									; Get memory size in 16 byte blocks
	inc ax										; Adjust AX
	add word [gs:TopSegment], ax				; Allocate memory
	pop gs
	pop ax
	iret

; --------------------------------------------------------------------------------------------
; Print sequence of characters from ESI, taking length from ECX.
; --------------------------------------------------------------------------------------------
; IN:  Inherited
; OUT: Inherited
; Special: Preserves registers.
ascii_dump:
	push ax
	push ecx
	push esi
	.loop:
		a32 o32 lodsb							; Byte from ESI
		push 0x01
		int 80h
		a32 o32 loop .loop						; Loop
	.done:
		pop esi
		pop ecx
		pop ax
		iret

; --------------------------------------------------------------------------------------------
; Make a beep from PC speaker. Recallibrating PIC!
; --------------------------------------------------------------------------------------------
; IN:  ECX --> Frequency
; OUT: None
; Special: Preserves registers.
beep:
	push eax
	push ebx
	push ecx
	push edx
	mov al, 0xB6
	out 0x43, al
	mov eax, 1193180
	xor edx, edx
	div ecx
	out 0x42, al								; Output low byte
	mov al, ah
	out 0x42, al								; Output high byte
	in al, 0x61									; Connect speaker to timer 2
	or al, 00000011b
	out 0x61, al
	pop edx
	pop ecx
	pop ebx
	pop eax
	iret

	
; --------------------------------------------------------------------------------------------
; Print string from ESI, centering it.
; --------------------------------------------------------------------------------------------
; IN & OUT: Inherited
; Special: Preserves registers.
center_print_string:
	push ax
	push bx
	push ecx
	push dx
	push 0x09
	int 80h										; Find length of string
	mov ax, 80									; 80 - length
	sub ax, cx
	xor dx, dx
	mov bx, 2
	div bx
	mov dx, ax
	push 0x0D
	int 80h
	mov ah, dl
	push 0x0E
	int 80h
	push 0x02
	int 80h
	.done:
		pop dx
		pop ecx
		pop bx
		pop ax
		iret

; ****************************OTHER KERNEL PARTS BEGIN FROM HERE***************************

break_int:
	push si
	push ds
	mov si, KernelSpace
	mov ds, si
	mov si, .msg
	push 0x02
	int 80h
	mov byte [BreakFlag], 0x01
	pop ds
	pop si
	iret
	.msg db 0x0A, 'Kernel: Aborting execution via CTRL+Break.', 0x0A, 0x00

; KERNEL DATA BLOCK Start
; ----------------------------------------
Version		db	'0.0.0.1 beta', 0x00
CurrentDrive			db	0x00
CurrentDirectoryCache	equ 0xC000
CurrentDirectoryPath	times 129 db 0x00
CurrentDirectoryCluster	dw	0x0000
CursorLocation			dw 0x0000
CharAttributes			db 0x07
CursorAttributes		db 0x70
CursorStatus			db 0x01
KernelSpace				equ	0xFFFF
TopSegment				dw	0x0050
TopMemory				dd	0x00500000
StackPointer			dd	0x00000000
StackSegment			dw	0x0000
FrameBuffer				equ	0x00110000
ClockTicks				dd	0x00000000
BreakFlag				db	0x00
MusicPlayingFlag		db	0x00
MusicRepeatFlag			db	0x00
MusicStartAddress		dd	0x00000000
MusicCurrentNote		dd	0x00000000
NoteLengthCounter		db	0x00
MusicSegment			dw	0x0000
CacheStatus				db	0x00
DriveInCache			db	0x00
SectorInCache			dw	0x0000
DiskCache				times 512 db 0x00
; ----------------------------------------
; KERNEL DATA BLOCK End

; KERNEL TIMER SECTION
; ---------------------------------------------
timer_int:
	push eax
	push gs
	mov ax, KernelSpace
	mov gs, ax
	inc dword [gs:ClockTicks]
	cmp byte [gs:MusicPlayingFlag], 0x01
	je .playmusic
	.done:
		pop gs
		pop eax
		iret
	.playmusic:
		push cx
		push esi
		push ds
		mov cx, word [gs:MusicSegment]
		mov ds, cx
		mov esi, dword [gs:MusicCurrentNote]
		xor cx, cx
		mov cl, byte [ds:esi]
		test cl, cl
		jz .music_end
		cmp byte [gs:NoteLengthCounter], cl
		je .next_note
		inc byte [gs:NoteLengthCounter]
	.playmusic_done:
		pop ds
		pop esi
		pop cx
		jmp .done
	.next_note:
		add dword [gs:MusicCurrentNote], 3
		mov byte [gs:NoteLengthCounter], 0
		mov esi, dword [gs:MusicCurrentNote]
		mov cx, word [ds:esi+1]
		test cx, cx
		jz .music_pause
		push 0x22
		int 80h
		jmp .playmusic_done
	.music_end:
		push 0x1E
		int 80h
		mov byte [gs:MusicPlayingFlag], 0x00
		mov byte [gs:NoteLengthCounter], 0
		jmp .playmusic_done
	.music_pause:
		push 0x1E
		int 80h
		jmp .playmusic_done
		
; --------------------------------------------------------------------
; |                         SYSTEM CALL BLOCK                        |
; --------------------------------------------------------------------
system_call:
	push ax
	push gs
	mov ax, KernelSpace
	mov gs, ax
	cmp byte [gs:BreakFlag], 0x01
	pop gs
	pop ax
	je .break_execution
	push bp
	mov bp, sp
	add bp, 8
	push ax
	push bx
	mov ax, word [ss:bp]
	sub bp, 2
	mov bx, word [ss:bp]
	add bp, 2
	mov word [ss:bp], bx
	sub bp, 4
	mov bx, word [ss:bp]
	add bp, 2
	mov word [ss:bp], bx
	sub bp, 4
	mov bx, word [ss:bp]
	add bp, 2
	mov word [ss:bp], bx
	sub bp, 2
	mov word [ss:bp], ax
	pop bx
	pop ax
	pop bp
	add sp, 2
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x00
	xchg sp, bp
	je terminate_process
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x01
	xchg sp, bp
	je put_char
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x02
	xchg sp, bp
	je print_string
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x03
	xchg sp, bp
	je new_line
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x04
	xchg sp, bp
	je string_to_integer
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x05
	xchg sp, bp
	je print_integer_hex
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x06
	xchg sp, bp
	je print_integer
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x07
	xchg sp, bp
	je input_integer
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x08
	xchg sp, bp
	je compare_strings
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x09
	xchg sp, bp
	je string_length
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x0A
	xchg sp, bp
	je initialise_screen
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x0B
	xchg sp, bp
	je disable_cursor
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x0C
	xchg sp, bp
	je enable_cursor
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x0D
	xchg sp, bp
	je get_cursor_position
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x0E
	xchg sp, bp
	je set_cursor_position
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x0F
	xchg sp, bp
	je center_print_string
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x10
	xchg sp, bp
	je input_string
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x11
	xchg sp, bp
	je set_palette
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x12
	xchg sp, bp
	je load_file
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x13
	xchg sp, bp
	je get_current_drive
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x14
	xchg sp, bp
	je start_new_program
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x15
	xchg sp, bp
	je lower_to_uppercase
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x16
	xchg sp, bp
	je upper_to_lowercase
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x17
	xchg sp, bp
	je get_current_palette
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x18
	xchg sp, bp
	je pause
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x19
	xchg sp, bp
	je allocate_memory
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x1A
	xchg sp, bp
	je cut_string
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x1B
	xchg sp, bp
	je ascii_dump
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x1C
	xchg sp, bp
	je get_char
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x1D
	xchg sp, bp
	je sleep
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x1E
	xchg sp, bp
	je stop_beep
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x1F
	xchg sp, bp
	je play_music
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x20
	xchg sp, bp
	je timer_read
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x21
	xchg sp, bp
	je load_dir
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x22
	xchg sp, bp
	je beep
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x23
	xchg sp, bp
	je floppy_read_sectors
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x24
	xchg sp, bp
	je floppy_read_byte
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x25
	xchg sp, bp
	je floppy_read_word
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x26
	xchg sp, bp
	je floppy_read_dword
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x27
	xchg sp, bp
	je string_copy
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x28
	xchg sp, bp
	je directory_scanner
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x29
	xchg sp, bp
	je set_current_drive
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x2A
	xchg sp, bp
	je stop_music
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x2B
	xchg sp, bp
	je ping_file
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x2C
	xchg sp, bp
	je fat_time_to_integer
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x2D
	xchg sp, bp
	je string_end
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x2E
	xchg sp, bp
	je get_current_dir
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x31
	xchg sp, bp
	je floppy_write_sectors
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x32
	xchg sp, bp
	je floppy_write_byte
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x33
	xchg sp, bp
	je floppy_write_word
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x34
	xchg sp, bp
	je floppy_write_dword
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x35
	xchg sp, bp
	je invalid_cache
	xchg sp, bp
	cmp word [ss:(bp-2)], 80h
	xchg sp, bp
	je enter_graphics_mode
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x81
	xchg sp, bp
	je draw_pixel
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x82
	xchg sp, bp
	je exit_graphics_mode
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x83
	xchg sp, bp
	je draw_line
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x84
	xchg sp, bp
	je draw_sprite
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x85
	xchg sp, bp
	je clear_frame_buffer
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x86
	xchg sp, bp
	je push_frame
	xchg sp, bp
	cmp word [ss:(bp-2)], 0x87
	xchg sp, bp
	je get_version_number
	xchg sp, bp
	cmp word [ss:(bp-2)], 0xA0
	xchg sp, bp
	je allocate_mem32
	mov ax, KernelSpace
	mov ds, ax
	mov si, .invalid_call_msg
	push 0x02
	int 80h
	push 0x00
	int 80h
	.invalid_call_msg		db	0x0A, "Phobis: An invalid system call has been issued by the program."
							db	0x0A, "        Execution aborted.", 0x0A, 0x00
	.break_execution:
		mov ax, KernelSpace
		mov ds, ax
		mov byte [BreakFlag], 0x00
		mov eax, 0xFFFFFFFE
		push 0x00
		int 80h

	A3			equ 220
	As3			equ 233
	B3			equ 247
	C3			equ 262
	Cs3			equ 277
	D3			equ 294
	Ds3			equ 311
	E3			equ 330
	F3			equ 349
	Fs3			equ 370
	G3			equ 392
	Gs3			equ 415
	A4			equ	440
	As4			equ 466
	B4			equ 494
	C4			equ 523
	Cs4			equ 554
	D4			equ	587
	Ds4			equ 622
	E4			equ 659
	F4			equ 698
	Fs4			equ 740
	G4			equ 784
	Gs4			equ 831
	A5			equ 880
	As5			equ 932
	B5			equ 988
	C5			equ 1047
	Cs5			equ 1109
	D5			equ 1175
	Ds5			equ 1245
	E5			equ 1319
	F5			equ 1397
	Fs5			equ 1480
	G5			equ 1568
	Gs5			equ 1661

times 0x8000-($-$$)			db 0x00				; Pad reserved sectors with 0x00
