from boom_birds_control.sih_guard import verify_sih_process


def test_requires_live_explicit_sih_process(tmp_path):
    assert not verify_sih_process(0, tmp_path)
    assert not verify_sih_process(123, tmp_path)
    process = tmp_path / "123"
    process.mkdir()
    executable = tmp_path / "PX4/build/px4_sitl_default/bin/px4"
    executable.parent.mkdir(parents=True)
    executable.touch()
    (process / "exe").symlink_to(executable)
    (process / "environ").write_bytes(b"PX4_SIM_MODEL=sihsim_quadx\0PX4_SIMULATOR=sihsim\0PX4_SYS_AUTOSTART=10040\0")
    assert verify_sih_process(123, tmp_path)
    (process / "environ").write_bytes(b"PX4_SIM_MODEL=real\0")
    assert not verify_sih_process(123, tmp_path)
