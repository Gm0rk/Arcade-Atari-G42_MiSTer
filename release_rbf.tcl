# ===========================================================================
#  Atari G42 for MiSTer -- release_rbf.tcl
#
#  Post-flow script for the release build. After a successful full
#  compilation of Arcade-Atari-G42 it moves the core out of output_files and
#  into releases, with the build date in its name:
#
#      output_files/Arcade-Atari-G42.rbf -> releases/Arcade-Atari-G42_YYYYMMDD.rbf
#
#  Arcade-Atari-G42.qsf runs it:
#      set_global_assignment -name POST_FLOW_SCRIPT_FILE "quartus_sh:release_rbf.tcl"
#  The debug build (Arcade-Atari-G42_debug.qsf) does not, and the script also
#  checks the revision name, so a debug core never lands in releases.
#
#  The date is the one sys/build_id.tcl wrote into build_id.v when this
#  compilation started, so the file name matches the version the OSD shows
#  (v260930 -> _20260930) even if the compile runs past midnight. A second
#  release build on the same day replaces that day's file.
#
#  Quartus runs post-flow scripts only after a successful flow; the script
#  still refuses to move a core whose flow report says the flow failed, or
#  one older than this compilation's synthesis report, so a stale .rbf is
#  never published.
#
#  Quartus passes quartus(args) = { <flow> <project> <revision> }, e.g.
#  { compile Arcade-Atari-G42 Arcade-Atari-G42 }, and runs the script in the
#  project folder.
# ===========================================================================

proc release_rbf {flow project revision} {
	set release_revision "Arcade-Atari-G42"
	set release_dir      "releases"

	# -----------------------------------------------------------------------
	#  Only the release build's full compilation
	# -----------------------------------------------------------------------
	if {$revision ne $release_revision} {
		post_message "release_rbf.tcl: $revision is not the release build;\
			nothing moved."
		return
	}
	if {$flow ne "compile"} {
		post_message "release_rbf.tcl: flow '$flow' is not a full compilation;\
			nothing moved."
		return
	}

	# -----------------------------------------------------------------------
	#  Output folder, from the project settings (output_files in the MiSTer
	#  template)
	# -----------------------------------------------------------------------
	set outdir "output_files"
	if {[project_exists $project]} {
		project_open $project -revision $revision
		set dir [get_global_assignment -name PROJECT_OUTPUT_DIRECTORY]
		if {$dir ne ""} { set outdir $dir }
		project_close
	}

	set rbf     [file join $outdir "$revision.rbf"]
	set flowrpt [file join $outdir "$revision.flow.rpt"]
	set maprpt  [file join $outdir "$revision.map.rpt"]

	# -----------------------------------------------------------------------
	#  Sanity checks: this compilation's core, from a flow that succeeded
	# -----------------------------------------------------------------------
	if {![file exists $rbf]} {
		post_message -type warning \
			"release_rbf.tcl: $rbf not found; nothing moved."
		return
	}

	if {[file exists $flowrpt]} {
		set f [open $flowrpt r]
		set report [read $f]
		close $f
		if {[regexp {Flow Status\s*;\s*Flow Failed} $report]} {
			post_message -type warning "release_rbf.tcl: the flow report says\
				the compilation failed; $rbf not moved."
			return
		}
	}

	if {[file exists $maprpt] && [file mtime $rbf] < [file mtime $maprpt]} {
		post_message -type warning "release_rbf.tcl: $rbf is older than\
			this compilation; not moved."
		return
	}

	# -----------------------------------------------------------------------
	#  Date: build_id.v's `define BUILD_DATE "YYMMDD", else today
	# -----------------------------------------------------------------------
	set date [clock format [clock seconds] -format %Y%m%d]
	if {[file exists "build_id.v"]} {
		set f [open "build_id.v" r]
		set text [read $f]
		close $f
		if {[regexp {BUILD_DATE\s+"(\d{6})"} $text -> yymmdd]} {
			set date "20$yymmdd"
		}
	}

	# -----------------------------------------------------------------------
	#  Move
	# -----------------------------------------------------------------------
	file mkdir $release_dir
	set dest [file join $release_dir "${revision}_${date}.rbf"]

	if {[catch {file rename -force $rbf $dest} err]} {
		post_message -type error \
			"release_rbf.tcl: could not move $rbf to $dest: $err"
		return
	}

	post_message "release_rbf.tcl: moved $rbf to $dest"
}

release_rbf [lindex $quartus(args) 0] [lindex $quartus(args) 1] \
	[lindex $quartus(args) 2]
