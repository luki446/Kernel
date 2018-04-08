
org 0x0100
bits 16												; Unreal mode
mov ax, 0xFFFF										; Reserve a 64kb segment of memory for file operations
push 0x19
int 80h
mov word [FileBuffer], cx							; Save segment
mov si, intro
push 0x02
int 80h
prompt_loop:
mov al, ''                     						; Draw prompt
push 0x01
int 80h
push 0x13                       					; Get current drive
int 80h
xor eax, eax
mov al, dl
xor cl, cl
xor dl, dl
push 0x06                       					; Print drive number
int 80h
mov al, ''
push 0x01
int 80h
mov di, CurrentDir
push 0x2E											; Get current dir
int 80h
mov si, CurrentDir
push 0x02											; Print current dir
int 80h
mov si, prompt										; Draw prompt
push 0x02
int 80h
mov bx, 0xFF										; Limit input to 0xFF characters
mov di, prompt_input								; Point to local buffer
push 0x10
int 80h												; Input string
push 0x03
int 80h												; New line
cmp byte [prompt_input], 0x00						; If no input, restart loop
je prompt_loop
extract_switches:
mov si, prompt_input								; Setup destination and source indexes
mov di, command_line_switches
.find_space_loop:
lodsb												; Byte from SI
cmp al, ' '											; Is it space?
je .get_switches									; If it is, save switches
test al, al											; Is it 0x00?
jz .no_switches										; If it is, write a 0x00 in the buffer, and quit
jmp .find_space_loop								; Otherwise loop
.get_switches:
mov byte [si-1], 0x00								; Add a terminator to the input
push 0x27
int 80h
jmp .done
.no_switches:
mov byte [di], 0x00
.done:
mov si, prompt_input
mov di, exit_msg									; Exit command
push 0x08
int 80h
cmp dl, 0x01
je exit_cmd
mov di, clear_msg									; Clear command
push 0x08
int 80h
cmp dl, 0x01
je clear_cmd
mov di, help_msg									; Help command
push 0x08
int 80h
cmp dl, 0x01
je help_cmd
mov di, ls_msg										; Ls command
push 0x08
int 80h
cmp dl, 0x01
je ls_cmd
mov di, dir_msg										; Dir (alias) command
push 0x08
int 80h
cmp dl, 0x01
je ls_cmd
mov di, cat_msg										; Cat command
push 0x08
int 80h
cmp dl, 0x01
je cat_cmd
mov di, cd_msg										; Cd command
push 0x08
int 80h
cmp dl, 0x01
je cd_cmd
mov di, time_msg									; Time command
push 0x08
int 80h
cmp dl, 0x01
je time_cmd
cmp dl, 0x01
je ver_cmd
mov di, color_msg									; Color command
push 0x08
int 80h
cmp dl, 0x01
je color_cmd
mov di, root_msg									; Root command
push 0x08
int 80h
cmp dl, 0x01
je root_cmd
mov si, prompt_input								; Prepare SI for start process function
mov di, command_line_switches						; Prepare to pass the switches
push 0x14
int 80h												; Try to start new process
cmp eax, 0xFFFFFFFF									; If fail, add .com and try again
jne prompt_loop										; Otherwise restart the loop
add_bin:
push 0x09
int 80h
cmp cx, 12
jg invalid_command
mov di, bin_added_buffer
push 0x27
int 80h
mov si, bin_added_buffer
.loop:
mov al, byte [ds:si]
test al, al
jz .add_bin
inc si
jmp .loop
.add_bin:
mov di, si
mov si, bin_msg
push 0x27
int 80h
mov si, bin_added_buffer							; Prepare SI for start process function
mov di, command_line_switches						; Prepare to pass the switches
push 0x14
int 80h												; Try to start new process
cmp eax, 0xFFFFFFFF									; If fail, print error message
jne prompt_loop										; Otherwise restart the loop
invalid_command:
mov si, not_found
push 0x02
int 80h
jmp prompt_loop
data:
intro					          db 0x0A, "Phobis v 1.0, Copyright (C) by Krzysztof Szewczyk. All rights reserved.", 0x0A
						          db 0x0A, 0x00
prompt			 		          db '$ ', 0x00
not_found	 			          db 'Bad command or filename.', 0x0A, 0x00
prompt_input          times 0x100 db 0x00
bin_added_buffer         times 13 db 0x00
command_line_switches times 0x100 db 0x00
bin_msg		                      db '.com', 0x00
CurrentDir              times 130 db 0x00
FileBuffer                        dw 0x0000
internal_commands:

exit_msg                          db 'exit',  0x00
clear_msg                         db 'cls',   0x00
help_msg                          db 'help',  0x00
ls_msg                            db 'ls',    0x00
cat_msg                           db 'cat',   0x00
cd_msg                            db 'cd',    0x00
time_msg                          db 'time',  0x00
ver_msg                           db 'ver',   0x00
dir_msg                           db 'dir',   0x00
color_msg                         db 'color', 0x00
root_msg                          db 'root',  0x00

time_cmd:
	push 0x20										; Read timer
	int 80h
	mov byte [.hours], ch							; Save timer values
	mov byte [.minutes], cl
	mov byte [.seconds], dh
	xor eax, eax									; Prepare print integer call: not left aligned, print at least 2 chars
	xor dl, dl
	mov cl, 2
	mov al, byte [.hours]							; Get and print hours
	push 0x06
	int 80h
	mov al, ':'										; Print separator
	push 0x01
	int 80h
	mov al, byte [.minutes]							; Get and print minutes
	push 0x06
	int 80h
	mov al, ':'										; Print separator
	push 0x01
	int 80h
	mov al, byte [.seconds]							; Get and print seconds
	push 0x06
	int 80h
	push 0x03										; New line
	int 80h
	jmp prompt_loop									; Return to prompt

	.hours		db	0x00
	.minutes	db	0x00
	.seconds	db	0x00

ver_cmd:
	mov edi, .version
	push 0x87										; Get version number
	int 80h
	mov esi, .version
	push 0x02
	int 80h
	push 0x03
	int 80h
	jmp prompt_loop									; Return to prompt
	.version times 16 db 0x00

cat_cmd:
	mov si, command_line_switches
	cmp byte [si], 0x00
	je .missing_parameter
	push es											; Set up target segment:offset
	mov ax, word [FileBuffer]
	mov es, ax
	xor bx, bx
	push 0x12										; Load file into buffer
	int 0x80
	pop es
	test dl, dl										; Check for failure
	jnz .failure
	push ds
	mov ax, word [FileBuffer]
	mov ds, ax
	xor si, si
	.loop:
		lodsb
		push 0x01
		int 0x80
		loop .loop
	pop ds
	jmp prompt_loop
	.missing_parameter:
		mov si, .missing_parameter_msg
		push 0x02
		int 0x80
		jmp prompt_loop
	.failure:
		mov si, .failure_msg
		push 0x02
		int 0x80
		jmp prompt_loop

	.missing_parameter_msg	db 'Missing filename.', 0x0A, 0x00
	.failure_msg			db 'File not found.', 0x0A, 0x00

cd_cmd:
	mov si, command_line_switches
	cmp byte [si], 0x00
	je .missing_parameter
	push 0x21										; Load directory
	int 0x80
	test dl, dl										; Check for failure
	jnz .failure
	jmp prompt_loop
	.missing_parameter:
		mov si, .missing_parameter_msg
		push 0x02
		int 0x80
		jmp prompt_loop
	.failure:
		mov si, .failure_msg
		push 0x02
		int 0x80
		jmp prompt_loop

	.missing_parameter_msg	db 'Missing directory name.', 0x0A, 0x00
	.failure_msg			db 'Directory not found.', 0x0A, 0x00

clear_cmd:
	push 0x0A
	int 0x80										; Clear screen
	jmp prompt_loop									; Return to prompt

color_cmd:
	mov si, command_line_switches
	cmp byte [si], 0x00
	je .missing_parameter
	push 0x17
	int 0x80
	mov bl, al
	push 0x04
	int 0x80
	mov ah, al
	mov al, bl
	push 0x11
	int 0x80
	jmp prompt_loop									; Return to prompt
	.missing_parameter:
		mov si, .missing_parameter_msg
		push 0x02
		int 0x80
		jmp prompt_loop
	.missing_parameter_msg	db 'Missing palette number (0-255).', 0x0A, 0x00
	
exit_cmd:
	xor eax, eax
	push 0x00
	int 0x80

help_cmd:
	mov si, .help_msg								; Display help message
	push 0x02
	int 0x80
	jmp prompt_loop									; Return to prompt
	.help_msg:
		db	"Phobis shell. Internal commands:", 0x0A
		db	0x0A
		db	" clear     --     Clear screen.", 0x0A
		db	" ls/dir    --     Lists files in the drive.", 0x0A
		db	" cd        --     Change the current working directory.", 0x0A
		db	" cat       --     Display content of a file.", 0x0A
		db	" time      --     Display current time.", 0x0A
		db  " color     --     Set text mode palette. (Use 'clear' to apply).", 0x0A
		db  " root      --     Set new root drive.", 0x0A
		db	" ver       --     Show current kernel version.", 0x0A
		db	" help      --     Show this list.", 0x0A
		db	" exit      --     Close the shell.", 0x0A
		db	0x00

ls_cmd:
	mov di, .FileName
	mov word [.EntryCounter], 0x0000
	mov	dword [.TotalSize], 0x00000000
	mov word [.FilesNumber], 0x0000
	mov word [.DirectoriesNumber], 0x0000
	.loop:
		mov ax, word [.EntryCounter]
		push 0x28
		int 0x80
		test dl, dl
		jnz .summary								; End of table, print summary
		inc word [.EntryCounter]
		mov dword [.Size], ecx						; Save size
		add dword [.TotalSize], ecx					; Increase total size
		mov byte [.Directory], dh
		cmp byte [.Directory], 0xFF					; Check if the entry is a directory
		jne .file_entry
		inc word [.DirectoriesNumber]
		jmp .convert_time
	.file_entry:
		inc word [.FilesNumber]
	.convert_time:
		push 0x2C									; Convert to standard time
		int 0x80
		mov byte [.Seconds], al
		mov byte [.Minutes], ah
		mov byte [.Hours], bl
		mov byte [.Days], bh
		mov byte [.Months], cl
		mov word [.Years], dx
		cmp byte [.Directory], 0xFF					; Check if the entry is a directory
		jne .skip_bracket1
		mov al, '['
		push 0x01
		int 0x80
	.skip_bracket1:
		mov si, .FileName							; Print file name
		push 0x02
		int 0x80
		cmp byte [.Directory], 0xFF					; Check if the entry is a directory
		jne .skip_bracket2
		mov al, ']'
		push 0x01
		int 0x80
	.skip_bracket2:
		push 0x0D
		int 0x80									; Get cursor coordinates
		mov ah, 15									; Set the X to 15
		push 0x0E
		int 0x80									; Set cursor coordinates
		cmp byte [.Directory], 0xFF					; Check if the entry is a directory
		je .skip_size
		xor cl, cl
		mov eax, dword [.Size]						; Print size, right align
		mov dl, 0x01
		push 0x06
		int 0x80
		mov si, .bytes_msg							; Print bytes
		push 0x02
		int 0x80
	.skip_size:
		push 0x0D
		int 0x80									; Get cursor coordinates
		mov ah, 35									; X = 35
		push 0x0E
		int 0x80									; Set cursor coordinates
		xor dl, dl									; Left align
		mov cl, 2
		xor eax, eax								; Print days
		mov al, byte [.Days]
		push 0x06
		int 0x80
		mov al, '/'									; Print separator
		push 0x01
		int 0x80
		xor eax, eax								; Print months
		mov al, byte [.Months]
		push 0x06
		int 0x80
		mov al, '/'									; Print separator
		push 0x01
		int 0x80
		mov cl, 4
		xor eax, eax								; Print years
		mov ax, word [.Years]
		push 0x06
		int 0x80
		push 0x0D
		int 0x80									; Get cursor coordinates
		add ah, 1									; X = X + 1
		push 0x0E
		int 0x80									; Set cursor coordinates
		mov cl, 2
		xor eax, eax								; Print hours
		mov al, byte [.Hours]
		push 0x06
		int 0x80
		mov al, ':'									; Print separator
		push 0x01
		int 0x80
		xor eax, eax								; Print minutes
		mov al, byte [.Minutes]
		push 0x06
		int 0x80
		mov al, ':'									; Print separator
		push 0x01
		int 0x80
		xor eax, eax								; Print seconds
		mov al, byte [.Seconds]
		push 0x06
		int 0x80
		push 0x03									; Next line
		int 0x80
		jmp .loop
	.summary:
		mov si, .totalbytes_msg
		push 0x02
		int 0x80
		mov eax, dword [.TotalSize]
		xor cl, cl
		xor dl, dl
		push 0x06
		int 0x80
		mov si, .dot_msg
		push 0x02
		int 0x80
		xor eax, eax
		mov ax, word [.FilesNumber]
		xor cl, cl
		xor dl, dl
		push 0x06
		int 0x80
		mov si, .files_msg
		push 0x02
		int 0x80
		xor eax, eax
		mov ax, word [.DirectoriesNumber]
		xor cl, cl
		xor dl, dl
		push 0x06
		int 0x80
		mov si, .dir_msg
		push 0x02
		int 0x80
		jmp prompt_loop
	.Directory		db	0x00
	.bytes_msg		db	' bytes', 0x00
	.totalbytes_msg	db	' Total bytes: ', 0x00
	.dot_msg		db	'.   ', 0x00
	.files_msg		db	' files, ', 0x00
	.dir_msg		db	' directories.', 0x0A, 0x00
	.EntryCounter	dw	0x0000
	.Seconds		db	0x00
	.Minutes		db	0x00
	.Hours			db	0x00
	.Days			db	0x00
	.Months			db	0x00
	.Years			dw	0x0000
	.Size			dd	0x00000000
	.TotalSize		dd	0x00000000
	.DirectoriesNumber	dw	0x0000
	.FilesNumber	dw	0x0000
	.FileName		times 13 db 0x00
	
root_cmd:
	mov si, command_line_switches
	cmp byte [si], 0x00
	je .missing_parameter
	push 0x04       							; String to integer
	int 0x80
	mov dl, al
	push 0x29      				 				; Set current drive
	int 0x80
	jmp prompt_loop								; Return to prompt
	.missing_parameter:
		mov si, .missing_parameter_msg
		push 0x02
		int 0x80
		jmp prompt_loop
	.missing_parameter_msg	db 'Required parameter missing: new root (integer).', 0x0A, 0x00
