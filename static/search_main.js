$(document).ready(function() {
    let x, y, z, w; // Goal Position 저장 변수

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
                                    <td>${response.position}</td>
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
        $.ajax({
            url: '/start-guide', 
            method: 'POST',
            contentType: 'application/json',
            data: JSON.stringify({ x: x, y: y, z: z, w: w }), // 저장된 Goal Position 값 전송
            success: function(response) {
                console.log('안내 시작:', response);
            },
            error: function() {
                console.log('안내 중 오류 발생');
            }
        });
    });
});
