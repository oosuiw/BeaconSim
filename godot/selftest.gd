extends Node
## UI 자체 시험 (--selftest=<폴더>): 실제 마우스·키 입력을 뷰포트에 주입해 사이드바·재생 바·단축키를 눌러 보고,
## 단계마다 결과를 출력(SELFTEST ...)하고 화면을 저장한다. 사용자 마우스는 건드리지 않는다.

var c: Node            # client.gd
var out := ""
var fails := 0

func run(client: Node, dir: String) -> void:
	c = client
	out = dir
	DirAccess.make_dir_recursive_absolute(out)
	# 0) 시작 화면: 개발 카드 → 개발 시작
	while c.mode_msg.is_empty():
		await c.get_tree().process_frame
	var tr := Time.get_ticks_msec()
	while not c.mode_msg.get("robot_ready", false) and Time.get_ticks_msec() - tr < 180000:   # 첫 실행: STL → 로봇 모델 생성
		await c.get_tree().process_frame
	await _sleep(0.5)
	_check("시작 화면 표시", c.start_ui.visible)
	await _shot("0_start")
	await _click(c.start_ui.cards["test"])
	await _sleep(0.5)
	_check("시험 카드 → 시험 설정 탭", c.start_ui.form.tab_btns.size() == 5)
	await _click(c.start_ui.cards["dev"])
	await _sleep(0.5)
	_check("개발 카드 선택", c.mode_msg["mode_sel"] == "dev" and c.start_ui.form.tab_btns.size() == 4)
	await _click(c.start_ui.start_btn)
	await _wait_world(-1)
	_check("개발 시작 → 시작 화면 닫힘", not c.start_ui.visible and c.mode_msg["mode"] == "dev")
	# 접이식 마스트 (있는 모델만): 접힌 채 시작 → U 키로 펴기
	await _sleep(1.0)
	if typeof(c.st.get("mast")) == TYPE_DICTIONARY:
		_check("마스트: 접힌 채로 시작해 자동으로 펴는 중", float(c.st["mast"]["frac"]) < 0.6 and int(c.st["mast"]["target"]) == 1)
		await _sleep(4.0)
		_check("마스트: 자동 펴짐", float(c.st["mast"]["frac"]) > 0.99)
		c._preset("beacon")
		await _shot("0_mast_deployed")
		await _key(KEY_U)
		await _sleep(5.0)
		_check("U 키 → 마스트 접힘", float(c.st["mast"]["frac"]) < 0.01)
		await _shot("0_mast_folded")
		await _key(KEY_U)
		await _sleep(4.5)
	await _sleep(2.0)
	# 1) 교통 탭 → '접근 차량 켜기' 끄기 → 교통량 슬라이더 비활성
	await _click(c.sidebar.tab_btns[2])
	var cb: CheckBox = c.sidebar.binds["traffic.enabled"][0]["w"]
	var was := cb.button_pressed
	await _click(cb)
	await _sleep(0.5)
	_check("교통 끄기 체크박스", cb.button_pressed == (not was) and c.sidebar.vals["traffic.enabled"] == (not was))
	var fl: HSlider = c.sidebar.binds["traffic.flow"][0]["w"]
	_check("교통량 슬라이더 비활성", fl.editable == (not was))
	await _shot("1_traffic_off")
	# 2) R → 재시작, 교통 없으면 주행 차량 0
	var seq: int = c.world_seq
	await _key(KEY_R)
	await _wait_world(seq)
	await _sleep(2.0)
	_check("R 재시작 → 새 월드", c.world_seq > seq)
	if was:
		_check("교통 꺼짐 → 주행 차량 0", int(c.st.get("n_active", -1)) == 0)
	# 다시 켜 둔다 (시험 뒤 설정 복원)
	await _click(cb)
	# 3) 시나리오 탭: 방향키 속도 슬라이더 끌기
	await _click(c.sidebar.tab_btns[1])
	var ks: HSlider = c.sidebar.binds["key.speed"][0]["w"]
	var before: float = ks.value
	await _drag(ks, 0.2, 0.9)
	await _sleep(0.6)
	_check("슬라이더 끌기 → 값 변경·백엔드 반영 (%.1f → %.1f)" % [before, ks.value],
		ks.value > before and absf(float(c.sidebar.vals["key.speed"]) - ks.value) < 1e-3)
	# 4) 선택 상자: 사고 위치 = 개활지 (팝업 항목 클릭 대신 선택 신호)
	var ob: OptionButton = c.sidebar.binds["scn.accident_where"][0]["w"]
	ob.select(1)
	ob.item_selected.emit(1)
	await _sleep(0.5)
	_check("선택 상자 → 값", c.sidebar.vals["scn.accident_where"] == "open")
	await _shot("2_scenario")
	# 5) 재생 바 P40 끄기 → 사이드바 보기 탭 체크박스도 같이 바뀜
	await _click(c.toggles["pandar"])
	await _sleep(0.3)
	var vb: CheckBox = c.sidebar.binds["view.pandar"][0]["w"]
	_check("재생 바 P40 끄기 → 점군 숨김·보기 탭 동기화", not c.show["pandar"] and not vb.button_pressed)
	# 6) 보기 탭: 범례 끄기, Pandar 색 = 정답 라벨
	await _click(c.sidebar.tab_btns[4])
	var lg: CheckBox = c.sidebar.binds["view.legend"][0]["w"]
	await _click(lg)
	await _sleep(0.3)
	_check("범례 끄기", not c.legend_box.visible)
	var seg: Array = c.sidebar.binds["view.pandar_color"][0]["bs"]
	await _click(seg[1])
	await _sleep(0.3)
	_check("Pandar 색 = 정답 라벨", c.pandar_color == 1)
	await _click(c.toggles["pandar"])
	await _shot("3_view")
	# 7) 일시정지 (Space), F1 도움말, Esc 닫기
	await _key(KEY_SPACE)
	await _sleep(0.5)
	_check("Space 일시정지", c.st.get("paused", false))
	await _key(KEY_SPACE)
	await _key(KEY_F1)
	_check("F1 도움말 열림", c.help_layer.visible)
	await _shot("4_help")
	await _key(KEY_ESCAPE)
	_check("Esc 도움말 닫힘", not c.help_layer.visible)
	# 8) 마우스 휠: 사이드바 위에서는 3D 확대 안 됨
	var d0: float = c.orbit["dist"]
	await _wheel(c.sidebar.get_global_rect().get_center())
	_check("사이드바 위 휠 → 3D 시점 그대로", absf(c.orbit["dist"] - d0) < 1e-6)
	d0 = c.orbit["dist"]
	await _wheel(_pt(c.playbar))
	_check("재생 바 위 휠 → 3D 시점 그대로", absf(c.orbit["dist"] - d0) < 1e-6)
	await _wheel(_pt(c.cam_panel))
	_check("카메라 패널 위 휠 → 3D 시점 그대로", absf(c.orbit["dist"] - d0) < 1e-6)
	await _wheel(_pt(c.sidebar.tab_grid))
	_check("사이드바 머리글 위 휠 → 3D 시점 그대로", absf(c.orbit["dist"] - d0) < 1e-6)
	var vs: Vector2 = c.get_viewport().get_visible_rect().size
	await _wheel(Vector2((vs.x + 380) / 2.0, vs.y / 2.0))
	_check("3D 위 휠 → 확대", c.orbit["dist"] < d0)
	# 원복: 범례·색
	await _click(seg[0])
	await _click(lg)
	# 9) 시험 모드: 홈 → 시험 카드 → 6초 시험 → 자동 종료 · 결과 창
	await _click(c.sidebar.home_btn)
	await _sleep(0.5)
	_check("홈 버튼 → 시작 화면", c.start_ui.visible and c.start_ui.back_btn.visible)
	await _click(c.start_ui.cards["test"])
	var tw := Time.get_ticks_msec()
	while c.mode_msg.get("mode_sel") != "test" and Time.get_ticks_msec() - tw < 5000:
		await c.get_tree().process_frame
	await _sleep(0.4)
	c._on_edit("test.dur_on", true)
	c._on_edit("test.duration", 6.0)
	await _sleep(0.4)
	_check("시험 시간 설정이 시작 화면 폼에 반영", absf(float(c.start_ui.form.vals.get("test.duration", 0)) - 6.0) < 1e-6)
	seq = c.world_seq
	await _click(c.start_ui.start_btn)
	await _wait_world(seq)
	await _sleep(1.0)
	_check("시험 시작 → 사이드바 시험 카드", c.sidebar.prog_lbl != null and c.mode_msg["mode"] == "test")
	_check("시험 모드 탭에 '시험' 탭", c.sidebar.tab_btns.size() == 6 and c.sidebar.tab_btns[5].text.contains("시험"))
	await _shot("5_test_running")
	var t0 := Time.get_ticks_msec()
	while not c.result_dlg.visible and Time.get_ticks_msec() - t0 < 30000:
		await c.get_tree().process_frame
	_check("시험 시간 종료 → 결과 창", c.result_dlg.visible)
	await _shot("6_result")
	await _key(KEY_ESCAPE)
	await _sleep(0.6)
	_check("Esc 결과 창 닫힘 · 사이드바 종료 카드", not c.result_dlg.visible and c.mode_msg.get("session", {}).get("done", false))
	# 10) 시험 중 조건 변경 → 잠금 배너 → 되돌리기
	c._on_edit("env.weather", "fog")
	await _sleep(1.0)
	_check("조건 변경 → 잠금 배너", c.mode_msg.get("changes", []).size() > 0 and c.sidebar.status_box.get_child_count() >= 2)
	await _shot("7_banner")
	c._on_action("revert")
	await _sleep(1.0)
	_check("되돌리기 → 배너 사라짐", c.mode_msg.get("changes", []).size() == 0)
	# 11) 회귀 화면
	await _click(c.sidebar.home_btn)
	await _sleep(0.4)
	await _click(c.start_ui.cards["regress"])
	await _sleep(1.0)
	_check("회귀 카드 → 시나리오 목록", c.start_ui.left_reg.visible and c.start_ui.reg_list.get_child_count() > 0)
	await _shot("8_regress")
	print("SELFTEST DONE fails=%d" % fails)
	c._send({"type": "quit"})
	c.get_tree().quit()

func _check(name: String, ok: bool) -> void:
	if not ok:
		fails += 1
	print("SELFTEST %s  %s" % ["OK  " if ok else "FAIL", name])

func _sleep(s: float) -> void:
	await c.get_tree().create_timer(s).timeout

func _wait_world(prev: int) -> void:
	var t0 := Time.get_ticks_msec()
	while c.world == null or c.world_seq == prev or c.st.is_empty() or int(c.st.get("world", -1)) != c.world_seq:
		if Time.get_ticks_msec() - t0 > 60000:          # 멈추지 말고 실패로 보고
			_check("새 장면 60 s 안에 도착", false)
			return
		await c.get_tree().process_frame

func _shot(name: String) -> void:
	await _sleep(0.4)
	await RenderingServer.frame_post_draw
	c.get_viewport().get_texture().get_image().save_png(out.path_join(name + ".png"))

## 화면 좌표 변환: 컨트롤 전역 좌표(UI 배율 적용 전) → 뷰포트 입력 좌표
func _pt(ctrl: Control, fx := 0.5) -> Vector2:
	var r := ctrl.get_global_rect()
	return Vector2(r.position.x + r.size.x * fx, r.position.y + r.size.y * 0.5)

func _push_mouse(pos: Vector2, pressed: bool, button := MOUSE_BUTTON_LEFT) -> void:
	var e := InputEventMouseButton.new()
	e.position = pos
	e.global_position = pos
	e.button_index = button
	e.pressed = pressed
	if pressed and button == MOUSE_BUTTON_LEFT:
		e.button_mask = MOUSE_BUTTON_MASK_LEFT
	c.get_viewport().push_input(e, true)

func _move(pos: Vector2, rel: Vector2, mask: int) -> void:
	var e := InputEventMouseMotion.new()
	e.position = pos
	e.global_position = pos
	e.relative = rel
	e.button_mask = mask
	c.get_viewport().push_input(e, true)

func _click(ctrl: Control) -> void:
	# 직전 조작으로 레이아웃이 바뀌었을 수 있음 (요약·탭 다시 그림) → 자리 잡을 때까지 몇 프레임 기다린 뒤 위치 계산
	for i in 3:
		await c.get_tree().process_frame
	if c.sidebar.scroll.is_ancestor_of(ctrl):
		c.sidebar.scroll.ensure_control_visible(ctrl)
		await c.get_tree().process_frame
	var p := _pt(ctrl)
	_move(p, Vector2.ZERO, 0)
	await c.get_tree().process_frame
	_push_mouse(p, true)
	await c.get_tree().process_frame
	_push_mouse(p, false)
	await c.get_tree().process_frame
	await c.get_tree().process_frame

func _drag(ctrl: Control, f0: float, f1: float) -> void:
	if c.sidebar.scroll.is_ancestor_of(ctrl):
		c.sidebar.scroll.ensure_control_visible(ctrl)
		await c.get_tree().process_frame
		await c.get_tree().process_frame
	var p0 := _pt(ctrl, f0)
	var p1 := _pt(ctrl, f1)
	_move(p0, Vector2.ZERO, 0)
	await c.get_tree().process_frame
	_push_mouse(p0, true)
	await c.get_tree().process_frame
	for i in range(1, 11):
		var p := p0.lerp(p1, i / 10.0)
		_move(p, (p1 - p0) / 10.0, MOUSE_BUTTON_MASK_LEFT)
		await c.get_tree().process_frame
	_push_mouse(p1, false)
	await c.get_tree().process_frame

func _wheel(pos: Vector2) -> void:
	_move(pos, Vector2.ZERO, 0)
	await c.get_tree().process_frame
	_push_mouse(pos, true, MOUSE_BUTTON_WHEEL_UP)
	_push_mouse(pos, false, MOUSE_BUTTON_WHEEL_UP)
	await c.get_tree().process_frame
	await c.get_tree().process_frame

func _key(k: Key) -> void:
	for pressed in [true, false]:
		var e := InputEventKey.new()
		e.keycode = k
		e.physical_keycode = k
		e.pressed = pressed
		c.get_viewport().push_input(e, true)
		await c.get_tree().process_frame
