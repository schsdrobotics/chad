package org.firstinspires.ftc.teamcode.util

interface System<COMMAND> {
	fun initialize()

	fun read()

	fun update(state: COMMAND)

	fun write()
}