# SPDX-License-Identifier: GPL-2.0-or-later

# aurix_program: put an application image as the toolchain produced it into
# AURIX program flash. Sourced by interface/tas_client.cfg, so that it is there
# for every AURIX board.

# Program flash as seen through the cached alias. The very same memory is also
# reachable with bit 29 set, and a vendor image routinely addresses one part of
# a bank through each of the two. OpenOCD resolves an image section to a flash
# bank by address, so the aliased sections land in no bank, and "flash
# write_image" only warns about a section it cannot place: erasing a bank to
# write the rest and then silently skipping the startup code that lives in the
# alias leaves a device that no longer boots.
set AURIX_PFLASH_START 0x80000000
set AURIX_PFLASH_END   0x88000000
set AURIX_PFLASH_ALIAS 0x20000000

# Naming for what an image may carry besides program flash, so that a skipped
# range is recognisable rather than just an address. A configuration may
# declare the regions of its own device as {start end name} triples; the
# families put them at different addresses, so nothing here may assume one.
if {![info exists AURIX_REGIONS]} {
	set AURIX_REGIONS {}
}

proc aurix_region_name {addr} {
	global AURIX_REGIONS

	foreach region $AURIX_REGIONS {
		if {$addr >= [lindex $region 0] && $addr <= [lindex $region 1]} {
			return [lindex $region 2]
		}
	}

	return "not program flash"
}

# Collapse byte ranges into named, readable spans.
proc aurix_merge_ranges {ranges} {
	set merged {}
	foreach r [lsort -index 0 -integer $ranges] {
		set start [lindex $r 0]
		set end [lindex $r 1]
		if {[llength $merged] && [lindex $merged end 1] >= $start} {
			if {$end > [lindex $merged end 1]} {
				set last [lindex $merged end]
				set merged [lreplace $merged end end \
					[list [lindex $last 0] $end]]
			}
		} else {
			lappend merged [list $start $end]
		}
	}

	set out_list {}
	foreach r $merged {
		lappend out_list [list [lindex $r 0] [lindex $r 1] \
			[aurix_region_name [lindex $r 0]]]
	}
	return $out_list
}

# Emit the extended linear address record a data record at $addr needs, and
# note what it set. Two readings of the upper address have to agree: the one
# of the format, the last such record ($ulba), and the one OpenOCD uses, the
# page the previous data record ended in ($run). A record that ends right on
# a 64K boundary moves the latter to the next page and not the former.
proc aurix_ihex_page {fout addr ulba_var run_var} {
	upvar $ulba_var ulba $run_var run

	set p [expr {$addr >> 16}]
	if {$p != $ulba || $p != $run} {
		# :02 0000 04 <page hi> <page lo> <checksum>
		set hi [expr {($p >> 8) & 0xFF}]
		set lo [expr {$p & 0xFF}]
		set sum [expr {(2 + 4 + $hi + $lo) & 0xFF}]
		puts $fout [format ":02000004%02X%02X%02X" \
			$hi $lo [expr {(-$sum) & 0xFF}]]
		set ulba $p
		set run $p
	}
}

# Rewrite an Intel hex file so that everything is addressed through the cached
# program flash alias, dropping whatever is not program flash.
#
# Folding clears bit 29, which lives in the upper half of the address, so the
# offset and the checksum of a data record never change: only the extended
# address records have to be reissued, and every data record is passed through
# untouched. They are all checked here first, as OpenOCD itself only ever gets
# to see the records that are kept.
#
# Returns a list of the ranges that were dropped, as {start end name} triples.
proc aurix_fold_ihex {in out} {
	global AURIX_PFLASH_START AURIX_PFLASH_END AURIX_PFLASH_ALIAS

	set fin [open $in r]
	set fout [open $out w]
	set ext 0
	set ulba -1
	set run -1
	set dropped {}
	set lineno 0
	set eof 0

	while {[gets $fin line] >= 0} {
		incr lineno
		set line [string trim $line]
		if {$line eq ""} continue

		set count -1
		if {[regexp {^:[0-9A-Fa-f]+$} $line]} {
			scan [string range $line 1 2] %x count
		}
		if {$count < 0 || [string length $line] != 11 + 2 * $count} {
			close $fin; close $fout
			error "$in:$lineno: not an Intel hex record"
		}
		set bytes [scan [string range $line 1 end] \
			[string repeat %2x [expr {$count + 5}]]]
		if {[expr [join $bytes +]] & 0xFF} {
			close $fin; close $fout
			error "$in:$lineno: checksum error"
		}
		lassign $bytes count off_hi off_lo type
		set offset [expr {($off_hi << 8) | $off_lo}]
		set data [lrange $bytes 4 end-1]

		switch -- $type {
			0 {
				set addr [expr {$ext + $offset}]
				set phys [expr {$addr & ~$AURIX_PFLASH_ALIAS}]

				if {$phys >= $AURIX_PFLASH_START && $phys < $AURIX_PFLASH_END} {
					aurix_ihex_page $fout $phys ulba run
					puts $fout $line
					set run [expr {($phys + $count) >> 16}]
				} else {
					lappend dropped [list $addr [expr {$addr + $count}]]
				}
			}
			1 {
				set eof 1
				break
			}
			2 - 4 {
				if {$count != 2} {
					close $fin; close $fout
					error "$in:$lineno: bad extended address record"
				}
				set base [expr {([lindex $data 0] << 8) | [lindex $data 1]}]
				set ext [expr {$type == 2 ? $base << 4 : $base << 16}]
			}
			3 - 5 { }
			default {
				close $fin; close $fout
				error "$in:$lineno: unsupported record type $type"
			}
		}
	}

	close $fin
	if {!$eof} {
		close $fout
		error "$in: no end of file record, the file is incomplete"
	}
	puts $fout ":00000001FF"
	close $fout

	return [aurix_merge_ranges $dropped]
}

# Same as aurix_fold_ihex, for an ELF. OpenOCD itself loads a PT_LOAD segment
# at its physical address unless every one of them is zero, so use p_paddr the
# same way and turn the loadable segments into an Intel hex.
#
# A Jim string indexes characters, so neither "read $fh $n" nor "string range"
# can be trusted on bytes above 0x7F: the file is turned into a hex string once
# and everything below slices that instead.
proc aurix_le16 {hex off} {
	scan [string range $hex [expr {$off * 2}] [expr {$off * 2 + 3}]] \
		%2x%2x b0 b1
	return [expr {$b0 | ($b1 << 8)}]
}

proc aurix_le32 {hex off} {
	scan [string range $hex [expr {$off * 2}] [expr {$off * 2 + 7}]] \
		%2x%2x%2x%2x b0 b1 b2 b3
	return [expr {$b0 | ($b1 << 8) | ($b2 << 16) | ($b3 << 24)}]
}

proc aurix_fold_elf {in out} {
	global AURIX_PFLASH_START AURIX_PFLASH_END AURIX_PFLASH_ALIAS

	# Not there in a jimtcl built with --minimal, as OpenOCD builds its own.
	if {[info commands binary] eq ""} {
		error "$in: reading an ELF needs the jimtcl binary extension, use the Intel hex file"
	}

	set fin [open $in rb]
	set raw [read $fin]
	close $fin
	binary scan $raw H* img
	set img [string tolower $img]
	set len [expr {[string length $img] / 2}]

	if {$len < 52 || [string range $img 0 7] ne "7f454c46"} {
		error "$in: not an ELF file"
	}
	# e_ident[EI_CLASS] == ELFCLASS32, e_ident[EI_DATA] == ELFDATA2LSB
	if {[string range $img 8 11] ne "0101"} {
		error "$in: only little endian 32 bit ELF is supported"
	}

	set phoff [aurix_le32 $img 28]
	set phentsize [aurix_le16 $img 42]
	set phnum [aurix_le16 $img 44]
	if {$phentsize < 32 || $phoff + $phnum * $phentsize > $len} {
		error "$in: program headers beyond the end of the file"
	}

	# collect the loadable segments first, so that p_paddr can be judged
	set segs {}
	set any_paddr 0
	for {set i 0} {$i < $phnum} {incr i} {
		set ph [expr {$phoff + $i * $phentsize}]
		if {[aurix_le32 $img $ph] != 1} continue
		set filesz [aurix_le32 $img [expr {$ph + 16}]]
		if {$filesz == 0} continue
		if {[aurix_le32 $img [expr {$ph + 4}]] + $filesz > $len} {
			error "$in: segment $i beyond the end of the file"
		}
		set paddr [aurix_le32 $img [expr {$ph + 12}]]
		if {$paddr != 0} { set any_paddr 1 }
		lappend segs [list [aurix_le32 $img [expr {$ph + 4}]] \
			[aurix_le32 $img [expr {$ph + 8}]] $paddr $filesz]
	}

	set fout [open $out w]
	set ulba -1
	set run -1
	set dropped {}

	foreach seg $segs {
		lassign $seg offset vaddr paddr filesz
		set base [expr {$any_paddr ? $paddr : $vaddr}]
		set phys [expr {$base & ~$AURIX_PFLASH_ALIAS}]

		if {$phys < $AURIX_PFLASH_START || $phys >= $AURIX_PFLASH_END} {
			lappend dropped [list $base [expr {$base + $filesz}]]
			continue
		}

		set pos 0
		while {$pos < $filesz} {
			set n [expr {$filesz - $pos}]
			if {$n > 16} { set n 16 }
			set addr [expr {$phys + $pos}]
			# a record may not straddle a 64K boundary
			set room [expr {0x10000 - ($addr & 0xFFFF)}]
			if {$n > $room} { set n $room }

			aurix_ihex_page $fout $addr ulba run

			set at [expr {($offset + $pos) * 2}]
			set chunk [string range $img $at [expr {$at + $n * 2 - 1}]]
			set sum [expr {$n + (($addr >> 8) & 0xFF) + ($addr & 0xFF)}]
			for {set k 0} {$k < $n * 2} {incr k 2} {
				scan [string range $chunk $k [expr {$k + 1}]] %x b
				incr sum $b
			}
			puts $fout [format ":%02X%04X00%s%02X" \
				$n [expr {$addr & 0xFFFF}] [string toupper $chunk] \
				[expr {(-$sum) & 0xFF}]]
			set run [expr {($addr + $n) >> 16}]
			incr pos $n
		}
	}

	puts $fout ":00000001FF"
	close $fout

	return [aurix_merge_ranges $dropped]
}

# Program an image into program flash.
#
# Takes the image exactly as the toolchain produced it, folds the non-cached
# alias onto the cached one so that no section is silently skipped, and leaves
# everything that is not program flash alone. In particular the user
# configuration blocks, which carry the boot mode headers and the protection
# settings, are never written: they are not part of an application update and
# a bad one can cost you the device.
#
# Like "program", it starts from "reset halt", so that no core is running from
# the flash being rewritten, and "run" starts the new image from reset.
proc aurix_program {filename args} {
	set verify 1
	set run 0
	foreach a $args {
		switch -- $a {
			verify { set verify 1 }
			noverify { set verify 0 }
			run { set run 1 }
			default { error "usage: aurix_program <file> \[noverify\] \[run\]" }
		}
	}

	set probe [open $filename rb]
	set magic [read $probe 4]
	close $probe

	# Created afresh, in TMPDIR if set; older jimtcl has no "file tempfile".
	if {[catch {file tempfile} folded]} {
		set folded "/tmp/openocd-aurix-[pid].hex"
	}

	# A command invoked from a proc returns its output instead of printing it.
	# The folded file goes whatever happens, also when folding fails halfway.
	set rc [catch {
		if {$magic eq "\x7FELF"} {
			set dropped [aurix_fold_elf $filename $folded]
		} else {
			set dropped [aurix_fold_ihex $filename $folded]
		}

		foreach d $dropped {
			echo [format "aurix_program: leaving 0x%08X - 0x%08X alone, %s" \
				[lindex $d 0] [expr {[lindex $d 1] - 1}] [lindex $d 2]]
		}

		reset halt
		echo [flash write_image erase $folded 0 ihex]
		if {$verify} {
			echo [verify_image $folded 0 ihex]
		}
	} err]
	file delete $folded
	if {$rc} {
		error $err
	}

	if {$run} {
		reset run
	}
}
