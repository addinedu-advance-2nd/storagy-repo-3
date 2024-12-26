$(document).ready(function() {
    let x, y, z, w, goal_position; // Goal Position 저장 변수

    $('.modal').css('display', 'none')

    // ! 사용자 등록 버튼 클릭 시 기능
    $(document).on('click', '.button-container__register-button', function() {
        console.log('사용자 등록 시작')
        $('.v_modal').css('display', 'block')

        $.ajax({
            url: '/register-user',
            type: 'POST',
            contentType: 'application/json',
            success: function(response) {
                if (response.status)
                    console.log('사용자 등록 성공:', response);
                    $('.modal').css('display', 'none')

                $('.button-container__register-button').prop('disabled', true);
            },
            error: function(error) {
                console.error('사용자 등록 실패:', error);
            }
        });
    })

    // ! 검색 버튼 클릭 시 기능
    $('.search-container__form').on('submit', function(event) {
        event.preventDefault(); // 기본 폼 제출 방지

        var searchTerm = $('.search-container__input').val();

        // searchTerm이 빈 문자열인 경우 요청을 보내지 않음
        if (searchTerm === "") {
            $('.result-container').empty().append('<div class="result-container__message">검색어를 입력하세요.</div>');
            return; // 함수 종료
        }

        $.ajax({
            url: '/search',
            method: 'GET',
            data: { query: searchTerm }, // 쿼리 파라미터로 검색어 전송
            success: function(response) {
                $('.result-container').empty(); // 이전 결과 비우기

                // 결과가 있을 경우
                if (Object.keys(response).length > 0) {
                    // 테이블 생성
                    let table = `
                        <table class="result-container__table">
                            <thead>
                                <tr>
                                    <th>선반</th>
                                    <th>이름</th>
                                    <th>위치</th>
                                </tr>
                            </thead>
                            <tbody>
                                <tr>
                                    <td>${response.section_name}</td>
                                    <td>${response.item_name}</td>
                                    <td>${response.item_num}</td>
                                </tr>
                            </tbody>
                        </table>
                        <button class="result-container__start-guide-button">안내<br>시작</button>`;
                    $('.result-container').append(table);

                    // Goal Position 값 저장
                    x = response.x;
                    y = response.y;
                    z = response.z;
                    w = response.w;
                    goal_position = response.goal_position_id
                } 
                // 결과가 없을 경우 메시지 표시
                else {
                    $('.result-container').append('<div class="result-container__message">검색결과가 없습니다.</div>');
                }
            },
            error: function() {
                // 에러 처리
                $('.result-container').empty().append('<div class="result-container__message">오류가 발생했습니다.</div>');
            }
        });
    });

    // ! 안내 시작 버튼 클릭 시 기능
    $(document).on('click', '.result-container__start-guide-button', function() {
        console.log(`x: ${x}, y: ${y}, z: ${z}, w: ${w}, goal_position: ${goal_position}`)

        console.log('주행 시작')
        $('.walking_modal').css('display', 'block')

        $.ajax({
            url: '/start-guide', 
            method: 'POST',
            contentType: 'application/json',
            data: JSON.stringify({ x: x, y: y, z: z, w: w, goal_position: goal_position }), // 저장된 Goal Position 값 전송
            success: function(response) {
                console.log('Navigation Status:', response.status);

                if (response.status == 2)
                    console.log('주행 성공')
                    $('.modal').css('display', 'none')

                    $('.search-container__input').val('')
                    $('.result-container').empty();
            },
            error: function() {
                console.log('주행 실패');
                $('.modal').css('display', 'none')
                $('.crying_modal').css('display', 'block')
            }
        });
    });

    // ! 계산대 이동 버튼 클릭 시 기능
    $(document).on('click', '.button-container__checkout-guide-button', function() {
        $('.search-container__input').val('')
        $('.result-container').empty();

        console.log('주행 시작')
        $('.money_modal').css('display', 'block')

        $.ajax({
            url: '/start-guide', 
            method: 'POST',
            contentType: 'application/json',
            data: JSON.stringify({ x: -0.8, y: -4.2, z: 0.72, w: -0.7, goal_position: 5 }), // 저장된 Goal Position 값 전송
            success: function(response) {
                console.log('Navigation Status:', response.status);

                if (response.status == 2)
                    console.log('주행 성공')
                    $('.modal').css('display', 'none')
            },
            error: function() {
                $('.modal').css('display', 'none')
                $('.crying_modal').css('display', 'block')
            }
        });
    });

    // ! 안내 종료 버튼 클릭 시 기능
    $(document).on('click', '.button-container__end-guide-button', function() {
        $('.search-container__input').val('')
        $('.result-container').empty();

        console.log('주행 시작')
        $('.tired_modal').css('display', 'block')

        $.ajax({
            url: '/end-guide', 
            method: 'POST',
            contentType: 'application/json',
            success: function(response) {
                console.log('Navigation Status:', response.status);

                if (response.status == 2)
                    console.log('주행 성공')
            },
            error: function() {
                $('.modal').css('display', 'none')
                $('.crying_modal').css('display', 'block')
            }
        });
    });
});
